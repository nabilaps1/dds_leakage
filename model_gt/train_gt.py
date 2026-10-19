
import argparse, json, os, random, time
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset, TensorDataset
from torch.utils.tensorboard import SummaryWriter


def load_windows(path, window, horizon, dt, v_max, w_max):
    df = pd.read_csv(path)
    t = np.arange(df["t"].min(), df["t"].max(), dt)          
    x = np.interp(t, df["t"], df["x"])
    y = np.interp(t, df["t"], df["y"])
    yaw = np.unwrap(np.interp(t, df["t"], df["yaw"]))        # avoids jumps

    v = np.hypot(np.diff(x), np.diff(y)) / dt                # linear velocity
    w = np.diff(yaw) / dt                                    # angular velocity

    X, Y = [], []
    for i in range(len(v) - window - horizon + 1):
        j = i + window - 1
        k = j + horizon
        dx, dy = x[k] - x[j], y[k] - y[j]
        c, s = np.cos(-yaw[j]), np.sin(-yaw[j])              # rotate to local frame
        X.append(np.stack([v[i:i+window], w[i:i+window]], -1))
        Y.append([c*dx - s*dy, s*dx + c*dy, yaw[k] - yaw[j]])
    X = np.array(X, np.float32) / np.array([v_max, w_max], np.float32)
    Y = np.array(Y, np.float32) / np.array([v_max*horizon*dt]*2 + [w_max*horizon*dt], np.float32)
    return X, Y, t[:len(X)]


def split_by_time(t0, n_blocks=10, val_blocks=2, seed=0):
    edges = np.linspace(t0.min(), t0.max()+1e-6, n_blocks+1)
    blocks = np.clip(np.digitize(t0, edges[1:-1]), 0, n_blocks-1)
    rng = np.random.RandomState(seed)
    val = set(rng.choice(n_blocks, val_blocks, replace=False))
    m = np.isin(blocks, list(val))
    return np.where(~m)[0], np.where(m)[0]


class RegressionModel(nn.Module):
    def __init__(self, d=64):
        super().__init__()
        self.inp = nn.Linear(2, d)
        self.gru = nn.GRU(d, d, batch_first=True)
        self.head = nn.Sequential(nn.Linear(d, d), nn.GELU(), nn.Linear(d, 3))

    def forward(self, x):
        out, _ = self.gru(self.inp(x))
        return self.head(out[:, -1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--device", default="cuda", choices=["cuda", "cpu"])
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--batch-size", type=int, default=128)
    ap.add_argument("--window", type=int, default=50)
    ap.add_argument("--horizon", type=int, default=10)
    ap.add_argument("--dt", type=float, default=0.02)
    ap.add_argument("--v-max", type=float, default=1.0)
    ap.add_argument("--w-max", type=float, default=2.0)
    ap.add_argument("--theta-weight", type=float, default=2.0)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    device = args.device if (args.device == "cpu" or torch.cuda.is_available()) else "cpu"
    os.makedirs(args.out, exist_ok=True)
    writer = SummaryWriter(os.path.join(args.out, "tb"))
    json.dump(vars(args), open(os.path.join(args.out, "args.json"), "w"), indent=2)

    X, Y, t0 = load_windows(args.poses, args.window, args.horizon, args.dt, args.v_max, args.w_max)
    tr_idx, va_idx = split_by_time(t0, seed=args.seed)
    ds = TensorDataset(torch.from_numpy(X), torch.from_numpy(Y))
    tr = DataLoader(torch.utils.data.Subset(ds, tr_idx), args.batch_size, shuffle=True, drop_last=True)
    va = DataLoader(torch.utils.data.Subset(ds, va_idx), args.batch_size)
    print(f"[data] {len(ds)} janelas | treino {len(tr_idx)} | val {len(va_idx)} | {device}")

    model = RegressionModel().to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=3e-3, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, args.epochs)
    huber = lambda p, y: (nn.functional.huber_loss(p[:, :2], y[:, :2])
                          + args.theta_weight * nn.functional.huber_loss(p[:, 2:], y[:, 2:]))

    best = float("inf")
    for ep in range(1, args.epochs+1):
        model.train(); loss_sum = 0.0; t_start = time.time()
        for xb, yb in tr:
            xb, yb = xb.to(device), yb.to(device)
            loss = huber(model(xb), yb)
            opt.zero_grad(); loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0); opt.step()
            loss_sum += loss.item() * len(xb)
        sched.step()

        model.eval(); ps, ys = [], []
        with torch.no_grad():
            for xb, yb in va:
                ps.append(model(xb.to(device)).cpu()); ys.append(yb)
        p, y = torch.cat(ps).numpy(), torch.cat(ys).numpy()
        xy = args.v_max * args.horizon * args.dt
        th = args.w_max * args.horizon * args.dt
        mae_xy = np.abs(p[:, :2]*xy - y[:, :2]*xy).mean()
        mae_th = np.abs(p[:, 2]*th - y[:, 2]*th).mean()
        print(f"epoch {ep:3d}/{args.epochs} | loss {loss_sum/len(tr_idx):.5f} | "
              f"val MAE_xy {mae_xy*100:.2f} cm | MAE_yaw {np.degrees(mae_th):.3f}° | {time.time()-t_start:.1f}s")
        writer.add_scalars("loss", {"train": loss_sum/len(tr_idx)}, ep)
        writer.add_scalars("val", {"mae_xy_m": mae_xy, "mae_yaw_rad": mae_th}, ep)

        if mae_xy + args.theta_weight*mae_th < best:
            best = mae_xy + args.theta_weight*mae_th
            torch.save({"model": model.state_dict(), "args": vars(args)},
                       os.path.join(args.out, "best.pt"))

    writer.close()
    print(f"[done] best score {best:.4f} -> {os.path.join(args.out, 'best.pt')}")


if __name__ == "__main__":
    main()

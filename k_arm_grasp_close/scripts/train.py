#!/usr/bin/env python3
"""Train the close-the-gripper model on make_dataset.py's output.

Writes ``<out>/model.pt`` (weights, config, feature names, normaliser: all
the node needs) and ``<out>/metrics.json``. With ``--cv`` it first
evaluates leave-one-bag-out (train on the other bags, run the decision on
the left-out one) and reports, per operator close:

* ``on time``: the decision fired within [close - lead_s - gap_s, close + after_s],
* ``early``: it fired before that,
* ``missed``: it did not fire by close + after_s,

and, for bags without any close, whether it ever fired (a false close).

    python3 train.py --data data --out runs/v1 --cv
"""

import argparse
import glob
import json
import os

import numpy as np
import torch
import yaml
from torch import nn

from k_arm_grasp_close.features import first_trigger
from k_arm_grasp_close.model import GraspCloseNet
from k_arm_grasp_close.model import save


def load(data_dir):
    with open(os.path.join(data_dir, 'config.yaml')) as f:
        cfg = yaml.safe_load(f)
    bags = []
    for path in sorted(glob.glob(os.path.join(data_dir, '*.npz'))):
        d = dict(np.load(path))
        d['name'] = os.path.splitext(os.path.basename(path))[0]
        bags.append(d)
    if not bags:
        raise RuntimeError('no .npz in {} (run make_dataset.py)'.format(data_dir))
    return cfg, bags


def full_window(valid, window):
    """True where the last ``window`` frames are all valid."""
    run = np.zeros(len(valid), int)
    n = 0
    for i, v in enumerate(valid):
        n = n + 1 if v else 0
        run[i] = n
    return run >= window


def samples(bag, window):
    """(windows (n, window, F), labels (n,)) of the labelled frames."""
    idx = np.flatnonzero(full_window(bag['valid'], window) & (bag['y'] >= 0))
    x = np.stack([bag['X'][i - window + 1:i + 1] for i in idx]) if len(idx) else \
        np.zeros((0, window, bag['X'].shape[1]), np.float32)
    return x, bag['y'][idx].astype(np.float32)


def train(bags, window, epochs, seed):
    xs, ys = zip(*[samples(b, window) for b in bags])
    x, y = np.concatenate(xs), np.concatenate(ys)
    if y.sum() == 0:
        raise RuntimeError('no positive frames: no operator close in the training bags')
    frames = np.concatenate([b['X'][b['valid']] for b in bags])
    mean = frames.mean(0)
    std = frames.std(0) + 1e-6
    x = (x - mean) / std
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    net = GraspCloseNet(x.shape[2])
    opt = torch.optim.Adam(net.parameters(), lr=1e-3, weight_decay=1e-4)
    # positives are few (lead_s + after_s per close): weigh them up
    loss_fn = nn.BCEWithLogitsLoss(pos_weight=torch.tensor((len(y) - y.sum()) / y.sum()))
    xt, yt = torch.from_numpy(x.astype(np.float32)), torch.from_numpy(y)
    for _ in range(epochs):
        net.train()
        for b in np.array_split(rng.permutation(len(y)), max(1, len(y) // 256)):
            opt.zero_grad()
            loss = loss_fn(net(xt[b]), yt[b])
            loss.backward()
            opt.step()
    net.eval()
    return net, mean, std


def predict(net, mean, std, bag, window):
    """Probability per frame, NaN where the window is not full."""
    ok = full_window(bag['valid'], window)
    p = np.full(len(ok), np.nan)
    idx = np.flatnonzero(ok)
    if len(idx):
        x = np.stack([bag['X'][i - window + 1:i + 1] for i in idx])
        with torch.no_grad():
            p[idx] = torch.sigmoid(net(torch.from_numpy(((x - mean) / std).astype(np.float32)))).numpy()
    return p


def judge(bag, prob, cfg):
    """Per close: outcome and dt (decision - close, s); for a bag without
    closes: whether the decision ever fired."""
    lab, dec = cfg['label'], cfg['decision']
    th, deb = float(dec['threshold']), int(dec['debounce'])
    early = float(lab['lead_s']) + float(lab['gap_s'])
    t, valid = bag['t'], ~np.isnan(prob)
    p = np.nan_to_num(prob)
    out = []
    start = t[0]
    for tc, to in bag['events']:
        sel = np.flatnonzero((t >= start) & (t <= tc + float(lab['after_s'])))
        k = first_trigger(p[sel], valid[sel], th, deb)
        if k is None:
            out.append(dict(outcome='missed', dt_s=None))
        else:
            dt = float(t[sel[k]] - tc)
            out.append(dict(outcome='early' if dt < -early else 'on time', dt_s=round(dt, 2)))
        start = (t[-1] if np.isnan(to) else to) + float(lab['reopen_settle_s'])
    if not len(bag['events']):
        k = first_trigger(p, valid, th, deb)
        out.append(dict(outcome='no close, fired' if k is not None else 'no close, quiet',
                        dt_s=None if k is None else round(float(t[k] - t[0]), 2)))
    return out


def frame_rates(bag, prob, cfg):
    th = float(cfg['decision']['threshold'])
    ok = ~np.isnan(prob)
    pos, neg = ok & (bag['y'] == 1), ok & (bag['y'] == 0)
    return int(pos.sum()), int((prob[pos] >= th).sum()), int(neg.sum()), int((prob[neg] >= th).sum())


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--data', required=True, help="make_dataset.py's --out")
    parser.add_argument('--out', required=True)
    parser.add_argument('--epochs', type=int, default=40)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--cv', action='store_true', help='leave-one-bag-out evaluation first')
    args = parser.parse_args()
    cfg, bags = load(args.data)
    window = int(round(float(cfg['window_s']) * float(cfg['rate_hz'])))
    names = [str(n) for n in bags[0]['names']]
    if any([str(n) for n in b['names']] != names for b in bags):
        raise RuntimeError('the .npz files have different features: rebuild them with one config')
    torch.set_num_threads(max(1, os.cpu_count() // 2))
    if not os.path.isdir(args.out):
        os.makedirs(args.out)
    metrics = dict(bags=[b['name'] for b in bags])

    if args.cv:
        rows, totals = [], np.zeros(4, int)
        for i, bag in enumerate(bags):
            rest = [b for j, b in enumerate(bags) if j != i]
            net, mean, std = train(rest, window, args.epochs, args.seed)
            prob = predict(net, mean, std, bag, window)
            fr = frame_rates(bag, prob, cfg)
            totals += fr
            for r in judge(bag, prob, cfg):
                rows.append(dict(bag=bag['name'], **r))
                print('{:32s} {:16s} {}'.format(bag['name'], r['outcome'],
                                                '' if r['dt_s'] is None else '{:+.2f} s'.format(r['dt_s'])))
        counts = {}
        for r in rows:
            counts[r['outcome']] = counts.get(r['outcome'], 0) + 1
        pos, pos_ok, neg, neg_fire = totals.tolist()
        metrics['cv'] = dict(per_close=rows, counts=counts,
                             frame_recall=round(pos_ok / max(pos, 1), 3),
                             frame_false_close=round(neg_fire / max(neg, 1), 3),
                             frames=dict(pos=pos, pos_ok=pos_ok, neg=neg, neg_fire=neg_fire))
        print('leave-one-bag-out: {}; frames: recall {:.3f}, false close {:.3f}'.format(
            counts, metrics['cv']['frame_recall'], metrics['cv']['frame_false_close']))

    net, mean, std = train(bags, window, args.epochs, args.seed)
    model_path = os.path.join(args.out, 'model.pt')
    save(model_path, net, cfg, names, mean, std)
    with open(os.path.join(args.out, 'metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=1)
    print('wrote {} ({} bags, {} features, window {})'.format(model_path, len(bags), len(names), window))


if __name__ == '__main__':
    main()

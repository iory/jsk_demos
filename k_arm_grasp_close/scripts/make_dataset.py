#!/usr/bin/env python3
"""Turn rosbags into feature frames and labels for train.py.

One ``<bag name>.npz`` per bag in ``--out``:

* ``t``: frame times (bag time, s), ``X``: frames (N, features),
* ``valid``: every input was fresh (``max_age_s``) at that frame,
* ``y``: 1 close now / 0 keep open / -1 not used (features.labels),
* ``events``: the operator's (close, reopen) times, reopen NaN if never.

A bag without any close is kept: all its frames are "keep open", which is
the data that teaches the model not to close.

    python3 make_dataset.py --out data bags/*.bag
"""

import argparse
import json
import os

import numpy as np
import rosbag
import yaml

from k_arm_grasp_close.features import FeatureBuilder
from k_arm_grasp_close.features import close_events
from k_arm_grasp_close.features import input_topics
from k_arm_grasp_close.features import labels

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config',
                              'right.yaml')


def convert(path, cfg):
    """One bag -> dict of arrays, or None (with the reason) if unusable."""
    bag = rosbag.Bag(path)
    inputs = input_topics(cfg)
    cmd = cfg['label']['command_topic']
    have = bag.get_type_and_topic_info().topics
    missing = [t for t in inputs if t not in have]
    if missing:
        return None, 'missing topics {}'.format(missing)
    if cmd not in have:
        return None, 'no {} (the label)'.format(cmd)
    fb = FeatureBuilder(cfg)
    names = fb.names
    period = 1.0 / float(cfg['rate_hz'])
    max_age = float(cfg['max_age_s'])
    index = int(cfg['label']['index'])
    t0, t1 = bag.get_start_time(), bag.get_end_time()
    latest = {}
    cmd_t, cmd_v = [], []
    ts, xs, valid = [], [], []
    next_t = t0 + period

    def tick(now):
        ok = all(t in latest and now - latest[t][0] <= max_age for t in inputs)
        if ok:
            xs.append(fb.frame(dict((t, latest[t][1]) for t in inputs)))
        else:
            fb.reset()
            xs.append(np.zeros(len(names), np.float32))
        ts.append(now)
        valid.append(ok)

    for topic, msg, bt in bag.read_messages(topics=inputs + [cmd]):
        t = bt.to_sec()
        while next_t <= t:
            tick(next_t)
            next_t += period
        if topic == cmd:
            cmd_t.append(t)
            cmd_v.append(float(msg.data[index]))
        else:
            latest[topic] = (t, msg)
    while next_t <= t1:
        tick(next_t)
        next_t += period
    ts = np.asarray(ts)
    events = close_events(cmd_t, cmd_v, float(cfg['label']['below']))
    y = labels(ts, events, cfg['label'], t1)
    ev = np.array([[c, np.nan if o is None else o] for c, o in events]).reshape(-1, 2)
    return dict(t=ts, X=np.asarray(xs, np.float32), valid=np.asarray(valid), y=y,
                events=ev, names=np.asarray(names)), None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bags', nargs='+')
    parser.add_argument('--out', required=True, help='directory for the .npz files')
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    args = parser.parse_args()
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    if not os.path.isdir(args.out):
        os.makedirs(args.out)
    with open(os.path.join(args.out, 'config.yaml'), 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    summary = []
    for path in args.bags:
        name = os.path.splitext(os.path.basename(path))[0]
        data, why = convert(path, cfg)
        if data is None:
            print('skip {}: {}'.format(name, why))
            continue
        np.savez_compressed(os.path.join(args.out, name + '.npz'), **data)
        row = dict(bag=name, frames=int(len(data['t'])), valid=int(data['valid'].sum()),
                   closes=int(len(data['events'])), positive=int((data['y'] == 1).sum()),
                   negative=int(((data['y'] == 0) & data['valid']).sum()))
        summary.append(row)
        print('{bag}: {frames} frames ({valid} valid), {closes} closes, '
              '{positive} positive / {negative} negative'.format(**row))
    with open(os.path.join(args.out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    print('{} bags -> {}'.format(len(summary), args.out))


if __name__ == '__main__':
    main()

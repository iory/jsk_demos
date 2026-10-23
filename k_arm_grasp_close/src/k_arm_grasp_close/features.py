"""Feature frames and labels, shared by make_dataset.py, train.py and the node.

One frame per tick (config ``rate_hz``) from the latest message of every
input topic, by receive / bag time (the robot's PCs are not clock-synced):

* every ToF range (sensor_msgs/Range): clipped to ``range_clip_m``, plus a
  valid flag (out of range / NaN -> the clip value, flag 0);
* every proximity intensity (std_msgs/UInt32) and every udts pixel
  (std_msgs/UInt32MultiArray): log(1 + value);
* the arm's joint velocities (difference to the previous frame) and, when
  ``joint_position`` is on, the joint angles.
"""

import numpy as np


def input_topics(cfg):
    """Every topic a frame needs, sensors first."""
    f = cfg['features']
    return list(f['ranges']) + list(f['intensities']) + list(f['arrays']) + [f['joint_states']]


class FeatureBuilder(object):
    """Latest messages -> one feature frame.

    Parameters
    ----------
    cfg : dict
        The config (config/right.yaml).
    """

    def __init__(self, cfg):
        self.f = cfg['features']
        self.rate_hz = float(cfg['rate_hz'])
        names = []
        for t in self.f['ranges']:
            names += [t + ':range', t + ':valid']
        names += [t + ':log_intensity' for t in self.f['intensities']]
        for t, n in self.f['arrays'].items():
            names += ['{}[{}]:log_intensity'.format(t, i) for i in range(n)]
        if self.f['joint_position']:
            names += list(self.f['joints'])
        if self.f['joint_velocity']:
            names += [j + ':vel' for j in self.f['joints']]
        self.names = names
        self.q_prev = None

    def reset(self):
        """Forget the previous joint angles (after an invalid frame)."""
        self.q_prev = None

    def frame(self, msgs):
        """One frame.

        Parameters
        ----------
        msgs : dict
            {topic: latest message} for every ``input_topics``.

        Returns
        -------
        numpy.ndarray
            float32, ``len(self.names)``.

        Raises
        ------
        ValueError
            A udts array of the wrong length, or joint states without a
            configured joint.
        """
        f = self.f
        clip = float(f['range_clip_m'])
        out = []
        for t in f['ranges']:
            m = msgs[t]
            r = float(m.range)
            ok = np.isfinite(r) and float(m.min_range) <= r <= float(m.max_range)
            out += [min(r, clip) if ok else clip, 1.0 if ok else 0.0]
        out += [np.log1p(float(msgs[t].data)) for t in f['intensities']]
        for t, n in f['arrays'].items():
            d = np.asarray(msgs[t].data, np.float64)
            if len(d) != n:
                raise ValueError('{} has {} values, the config says {}'.format(t, len(d), n))
            out += list(np.log1p(d))
        js = msgs[f['joint_states']]
        pos = dict(zip(js.name, js.position))
        missing = [j for j in f['joints'] if j not in pos]
        if missing:
            raise ValueError('{} lacks {}'.format(f['joint_states'], missing))
        q = np.array([pos[j] for j in f['joints']])
        dq = np.zeros(len(q)) if self.q_prev is None else (q - self.q_prev) * self.rate_hz
        self.q_prev = q
        if f['joint_position']:
            out += list(q)
        if f['joint_velocity']:
            out += list(dq)
        return np.asarray(out, np.float32)


def close_events(times, values, below):
    """The operator's closes in one bag.

    Parameters
    ----------
    times : sequence of float
        Message times of the effort command.
    values : sequence of float
        Its data[index].
    below : float
        A value below this is closing.

    Returns
    -------
    list of (float, float or None)
        (close time, reopen time or None if it stays closed).
    """
    events = []
    closed = False
    for t, v in zip(times, values):
        if not closed and v < below:
            events.append([t, None])
            closed = True
        elif closed and v >= below:
            events[-1][1] = t
            closed = False
    return [tuple(e) for e in events]


def labels(times, events, label_cfg, end_time):
    """Per frame 1 (close now), 0 (keep open) or -1 (not used).

    Parameters
    ----------
    times : numpy.ndarray
        Frame times.
    events : list of (float, float or None)
        ``close_events``.
    label_cfg : dict
        The config's ``label`` section.
    end_time : float
        End of the bag (for a close that is never reopened).

    Returns
    -------
    numpy.ndarray
        int8 per frame.
    """
    lead, after = float(label_cfg['lead_s']), float(label_cfg['after_s'])
    gap, settle = float(label_cfg['gap_s']), float(label_cfg['reopen_settle_s'])
    y = np.zeros(len(times), np.int8)
    for tc, to in events:
        reopen = end_time if to is None else to
        y[(times >= tc - lead - gap) & (times <= reopen + settle)] = -1
    for tc, _ in events:
        y[(times >= tc - lead) & (times <= tc + after)] = 1
    return y


def first_trigger(prob, valid, threshold, debounce):
    """Index where the decision fires (``debounce`` consecutive valid frames
    at or above ``threshold``), or None."""
    run = 0
    for i, (p, v) in enumerate(zip(prob, valid)):
        run = run + 1 if (v and p >= threshold) else 0
        if run >= debounce:
            return i
    return None

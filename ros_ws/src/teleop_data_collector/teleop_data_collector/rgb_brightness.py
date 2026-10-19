"""Pre-recording RGB readiness checks using the existing zero-copy Image pixel view."""
import math
import numpy as np


def validate_policy(threshold, duration_sec):
    if not math.isfinite(threshold) or not 0 <= threshold <= 255:
        raise ValueError('RGB brightness threshold must be within 0..255')
    if not math.isfinite(duration_sec) or duration_sec <= 0:
        raise ValueError('RGB readiness duration must be positive')


def mean_rgb(image):
    if image.encoding.lower() not in {'rgb8', 'bgr8'}:
        raise ValueError(f'RGB亮度校验不支持编码：{image.encoding}')
    h, w, step = image.height, image.width, image.step
    if h <= 0 or w <= 0 or step < w * 3:
        raise ValueError('RGB亮度校验：无效图像尺寸或行步长')
    pixels = np.frombuffer(image.data, dtype=np.uint8)
    if len(pixels) != h * step:
        raise ValueError('RGB亮度校验：图像数据长度不匹配')
    # Padding is not image content. uint64 accumulation prevents overflow.
    return float(pixels.reshape(h, step)[:, :w * 3].sum(dtype=np.uint64)) / (h * w * 3)


class RgbReadiness:
    """Idle-only brightness gate. Does not produce episode validation failures."""

    def __init__(self, topics, *, threshold=25.0, stable_sec=3.0, max_gap_sec=.15):
        validate_policy(threshold, stable_sec)
        self.topics = tuple(topics)
        self.threshold = threshold
        self.stable_sec = stable_sec
        self.max_gap_sec = max_gap_sec
        self.paused = False
        self.reset()

    def reset(self):
        self._states = {topic: dict(last=None, source=None, since=None, mean=None, error='等待图像')
                        for topic in self.topics}

    def set_paused(self, paused):
        if paused != self.paused:
            self.paused = paused
            self.reset()

    def observe(self, topic, header_ns, now, mean, error=None):
        if self.paused or topic not in self._states:
            return
        state = self._states[topic]
        continuous = (state['last'] is not None and 0 <= now-state['last'] <= self.max_gap_sec
                      and header_ns is not None and state['source'] is not None
                      and 0 < header_ns-state['source'] <= self.max_gap_sec*1e9)
        if not continuous:
            state['since'] = None
        state.update(last=now, source=header_ns, mean=mean)
        if error or header_ns is None or header_ns <= 0 or mean is None or not math.isfinite(mean):
            state.update(since=None, error=error or '图像或时间戳无效')
        elif mean < self.threshold:
            state.update(since=None, error=f'平均像素值 {mean:.1f} < {self.threshold:g}/255')
        else:
            if state['since'] is None:
                state['since'] = now
            state['error'] = ''

    def snapshot(self, now):
        waiting, streams = [], {}
        for topic, state in self._states.items():
            fresh = state['last'] is not None and 0 <= now-state['last'] <= self.max_gap_sec
            duration = max(0., state['last']-state['since']) if state['since'] is not None else 0.
            ready = not self.paused and fresh and not state['error'] and duration >= self.stable_sec
            detail = state['error'] or f'亮度正常 {duration:.1f}/{self.stable_sec:g} 秒'
            if not fresh:
                detail = '等待新鲜图像'
            streams[topic] = dict(ready=ready, mean=state['mean'], normal_sec=duration, reason=detail)
            if not ready:
                waiting.append(f'{topic}：{detail}')
        return dict(ready=not waiting, threshold=self.threshold, stable_sec=self.stable_sec,
                    paused=self.paused, streams=streams,
                    message=('录制前亮度检查：' + '；'.join(waiting)) if waiting else '')

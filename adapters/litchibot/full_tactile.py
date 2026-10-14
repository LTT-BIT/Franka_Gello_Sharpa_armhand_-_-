"""Full MANUS telemetry: SDK reads on owner thread, image work off-thread."""
import time
from array import array
from types import SimpleNamespace

from .normal_telemetry import DeferredTelemetry


def fast_tactile_image(image,*,frame_id,stamp):
    from sharpa_driver.messages import tactile_image_message
    # Generated ROS setters validate bytes one element at a time. A typed byte
    # array takes their fast path and produces the identical serialized image.
    packed=SimpleNamespace(height=image.height,width=image.width,encoding=image.encoding,
        step=image.step,data=array('B',image.data))
    return tactile_image_message(packed,frame_id=frame_id,stamp=stamp)


class FullTactilePublisher:
    def __init__(self,node):
        self.node=node
        self.worker=DeferredTelemetry()

    def poll(self):
        from sharpa_driver.messages import validate_tactile_route
        for (side,finger),publishers in self.node._tactile_publishers.items():
            try:
                frame=self.node._hands[side].read_tactile(finger,timeout_s=0.0)
                if frame is None:continue
                stamp=self.node.get_clock().now().to_msg()
                frame_id=validate_tactile_route(frame,expected_side=side,expected_finger=finger)
                # TactileImage.data is immutable bytes; retain the acquisition
                # stamp. Coalesce each finger instead of queuing stale images.
                self.worker.submit((side.value,finger.value),
                    lambda f=frame,p=publishers,s=stamp,i=frame_id:self.publish(f,p,s,i))
            except Exception as error:
                now=time.monotonic()
                if now-self.node._last_tactile_warning_at[side]>2:
                    self.node.get_logger().error(f'failed to collect {side.value} tactile: {error}')
                    self.node._last_tactile_warning_at[side]=now

    def publish(self,frame,publishers,stamp,frame_id):
        from sharpa_driver.messages import tactile_wrench_message
        try:
            for name,image in (('raw',frame.raw),('deformation',frame.deformation)):
                if image is not None:
                    publishers[name].publish(fast_tactile_image(image,frame_id=frame_id,stamp=stamp))
            if frame.wrench:
                publishers['wrench'].publish(tactile_wrench_message(frame,stamp=stamp))
        except Exception as error:
            self.node.get_logger().error(f'failed to publish full-mode tactile: {error}')

    def close(self):
        self.worker.close()

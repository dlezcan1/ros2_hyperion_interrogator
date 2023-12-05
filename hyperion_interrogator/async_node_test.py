import asyncio
import random
import time
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import (
    Executor,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.callback_groups import (
    ReentrantCallbackGroup,
)
from rclpy.node import Node
import rclpy.parameter
import rclpy.task
import rclpy.timer

from std_msgs.msg import (
    String,
)
from std_srvs.srv import (
    Trigger,
)


class AsyncNode(Node):
    def __init__(self, node_name: str = "AsyncNode"):
        super().__init__(node_name=node_name)
        self.counter_pubs    = 0
        self.counter_trigger = 0
        self.asyncio_queue   = asyncio.Queue(maxsize=20)
        self.stream          = False

        self.task_spin  : asyncio.Task = None
        self.task_queue : asyncio.Task = None
        self.task_pub   : asyncio.Task = None

        self.sleep_period_pub   = self.declare_parameter(
            "asyncio.sleep_period.publisher",
            0.001,
        ).get_parameter_value().double_value
        self.sleep_period_queue = self.declare_parameter(
            "asyncio.sleep_period.queue",
            0.01,
        ).get_parameter_value().double_value
        self.max_triggers = self.declare_parameter(
            "max_triggers",
            5,
        ).get_parameter_value().integer_value

        self.pub = self.create_publisher(
            String,
            "chatter",
            10,
        )
        self.srv = self.create_service(
            Trigger,
            "trigger",
            self.callback_service,
        )

    # __init__

    def callback_service(self, request: Trigger.Request, response: Trigger.Response):
        if not self.stream:
            response.success = False
            response.message = "Streamer not running yet!"

        # if
        self.counter_trigger += 1

        response.success = True
        response.message = f"Counter trigger now at: {self.counter_trigger}"
        
        if self.counter_trigger >= self.max_triggers:
            self.stream = False

        # if

        return response
    
    # callback_service

    def configure_executor(self, executor: Executor):
        executor.add_node(self)
        self.task_queue = executor.create_task(self.queue_random_number)
        self.task_pub   = executor.create_task(self.publish_chatter)

        return executor
    
    # configure_executor

    async def queue_random_number(self):
        self.get_logger().info("Starting to queue data.")
        while rclpy.ok():
            num = random.randint(0, 100)
            self.get_logger().debug(f"[QUEUE_RANDOM_NUMBER]: Queueing number {num}")
            await self.asyncio_queue.put(num)
            self.get_logger().debug(f"[QUEUE_RANDOM_NUMBER]: Enqueued number {num}. # Queued: {self.asyncio_queue.qsize()}")

            if self.sleep_period_queue > 0:
                self.get_logger().debug(f"[QUEUE_RANDOM_NUMBER]: Sleeping for {self.sleep_period_queue} seconds")
                await asyncio.sleep(self.sleep_period_queue)
                self.get_logger().debug(f"[QUEUE_RANDOM_NUMBER]: Finished sleeping for {self.sleep_period_queue} seconds")

            # if

        # while
        self.get_logger().info("Stopped queueing data.")

        self.asyncio_queue.join()

    # queue_random_number

    async def publish_chatter(self):
        self.stream = True
        self.get_logger().info("Starting to publish chatter.")
        while rclpy.ok() and self.stream:
            self.get_logger().debug(f"[PUBLISH_CHATTER]: Awaiting queue data")
            random_number = await self.asyncio_queue.get()
            self.get_logger().debug(f"[PUBLISH_CHATTER]: Received queue data: {random_number}")
            self.asyncio_queue.task_done()

            msg = String(
                data=f"Hello world {self.counter_pubs}! Triggers: {self.counter_trigger}, Data: {random_number}"
            )

            self.get_logger().debug(f"[PUBLISH_CHATTER]: Publishing: {msg.data}")

            self.pub.publish(msg)

            if self.sleep_period_pub > 0:
                self.get_logger().debug(f"[PUBLISH_CHATTER]: Sleeping for {self.sleep_period_pub} seconds")
                await asyncio.sleep(self.sleep_period_pub)
                self.get_logger().debug(f"[PUBLISH_CHATTER]: Finished sleeping for {self.sleep_period_pub} seconds")

            # if

            self.counter_pubs += 1

        # while
        self.get_logger().info("Stopped publishing chatter.")

    # publish_chatter

    def shutdown(self):
        self.stream = False
        self.task_spin.cancel()
        self.task_pub.cancel()
        self.task_queue.cancel()

        return (
            not self.stream
            and self.task_spin.cancelled()
            and self.task_pub.cancelled()
            and self.task_queue.cancelled()
        )

    # shutdown

    async def spin(self):
        cancel = self.create_guard_condition(lambda: None)
        def _spin(node: Node, future: asyncio.Future, event_loop: asyncio.AbstractEventLoop):
            while not future.cancelled():
                rclpy.spin_once(node)

            if not future.cancelled():
                event_loop.call_soon_threadsafe(future.set_result, None)

        # _spin

        event_loop  = asyncio.get_event_loop()
        spin_task   = event_loop.create_future()
        spin_thread = threading.Thread(
            target=_spin,
            args=(self, spin_task, event_loop)
        )
        spin_thread.start()

        try:
            await spin_task
        except asyncio.CancelledError:
            cancel.trigger()

        spin_thread.join()
        self.destroy_guard_condition(cancel)

    # spin

    async def run(self):
        self.task_spin  = asyncio.get_event_loop().create_task(self.spin())
        self.task_queue = asyncio.get_event_loop().create_task(self.queue_random_number())
        self.task_pub   = asyncio.get_event_loop().create_task(self.publish_chatter())

        tasks = [
            self.task_spin,
            self.task_queue,
            self.task_pub,
        ]
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)

        # cancel the rest of the tasks
        for task in tasks:
            if task.cancel():
                await task

        # for

    # run
        


# class: AsyncNode


def main(args=None):
    rclpy.init(args=args)

    async_node = AsyncNode()
    try:
        asyncio.get_event_loop().run_until_complete(async_node.run())
    
    except asyncio.CancelledError:
        pass
    
    finally:
        asyncio.get_event_loop().close()

    rclpy.shutdown()

# main

if __name__ == "__main__":
    main()


# if __main__


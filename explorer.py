# LEGO type:advanced slot:0 autostart

# Build: https://www.onekitprojects.com/51515/explorer

import hub
import runtime
from util import movement
from util import sensors


class LedColor:
    Nothing = -1
    Black = 0
    Pink = 1
    Purple = 2
    Blue = 3
    Azure = 4
    Cyan = 5
    Green = 6
    Yellow = 7
    Orange = 8
    Red = 9
    White = 10


class Robot:
    def __init__(self, vm):
        self.vm = vm

    def move_start_forward(self):
        hub.led(LedColor.Green)
        movement.move_start(
            self.vm, movement.from_direction("forward", self.vm.store.move_speed())
        )

    async def move_cm_async(self, direction, cm):
        await movement.move_degrees(
            self.vm,
            (cm / self.vm.store.move_calibration()) * 360,
            movement.from_direction(direction, self.vm.store.move_speed()),
        )

    async def move_degrees_async(self, direction, degrees):
        await movement.move_degrees(
            self.vm,
            degrees,
            movement.from_direction(direction, self.vm.store.move_speed()),
        )

    def stop_movement(self):
        movement.move_stop(self.vm)

    async def run_motor_to_position(self, motor_port, direction, position):
        speed = abs(self.vm.store.motor_speed(motor_port))
        (acceleration, deceleration) = self.vm.store.motor_acceleration(motor_port)
        stall = self.vm.store.motor_stall(motor_port)
        stop = self.vm.store.motor_stop(motor_port)
        motor = self.vm.system.motors.on_port(motor_port)
        self.vm.store.motor_last_status(
            await motor.run_to_position_async(
                position,
                speed,
                direction,
                stall,
                stop,
                acceleration,
                deceleration,
            )
        )

    async def rotate_radar_async(self, direction, position):
        await self.run_motor_to_position("E", direction, position)

    def get_color(self):
        result = sensors.get_sensor_value("C", 0, LedColor.Nothing, (61,))
        return LedColor.Nothing if result is None else result

    def get_proximity(self):
        max_dist = 200
        result = sensors.get_sensor_value("D", 0, max_dist, (62,))
        return max_dist if result is None else result

    async def turn_async(self):
        await self.rotate_radar_async("counterclockwise", 270)
        yield 100
        left_distance = self.get_proximity()

        await self.rotate_radar_async("clockwise", 90)
        yield 100
        right_distance = self.get_proximity()

        hub.led(LedColor.Black)
        await self.rotate_radar_async("shortest", 0)

        if left_distance > right_distance:
            await self.move_degrees_async("counterclockwise", 155)
        else:
            await self.move_degrees_async("clockwise", 155)

        self.move_start_forward()

    async def run(self):
        self.vm.store.move_pair(("A", "B"))
        self.vm.store.move_speed(50)

        await self.rotate_radar_async("shortest", 0)
        self.move_start_forward()

        while True:
            if self.get_color() == LedColor.Azure:
                hub.led(LedColor.Red)
                await self.move_cm_async("back", 10)
                hub.led(LedColor.Black)
                await self.turn_async()

            if self.get_proximity() < 15:
                self.stop_movement()
                hub.led(LedColor.Yellow)
                if self.get_proximity() < 10:
                    await self.move_cm_async("back", 10 * self.get_proximity())
                await self.turn_async()

            yield


async def main(vm, stack):
    robot = Robot(vm)
    await robot.run()
    vm.stop()


def setup(rpc, system, stop):
    vm = runtime.VirtualMachine(rpc, system, stop, None)
    vm.register_on_start(None, main)
    return vm

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


class VirtualMachineExt(runtime.VirtualMachine):
    def __init__(self, rpc, system, stop):
        super().__init__(rpc, system, stop, None)

    async def move_cm_async(self, direction: str, cm: float) -> None:
        await self.move_degrees_async(
            direction, cm / self.store.move_calibration() * 360
        )

    async def move_degrees_async(self, direction: str, degrees: float) -> None:
        await movement.move_degrees(
            self,
            degrees,
            movement.from_direction(direction, self.store.move_speed()),
        )

    def start_move_direction(self, direction: str) -> None:
        movement.move_start(
            self, movement.from_direction(direction, self.store.move_speed())
        )

    def stop_movement(self) -> None:
        movement.move_stop(self)

    async def run_motor_to_position(
        self, motor_port: str, direction: str, position: float
    ) -> None:
        speed = abs(self.store.motor_speed(motor_port))
        (acceleration, deceleration) = self.store.motor_acceleration(motor_port)
        stall = self.store.motor_stall(motor_port)
        stop = self.store.motor_stop(motor_port)
        motor = self.system.motors.on_port(motor_port)
        self.store.motor_last_status(
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

    def get_color(self, sensor_port: str) -> int:
        result = sensors.get_sensor_value(sensor_port, 0, LedColor.Nothing, (61,))
        return LedColor.Nothing if result is None else result

    def get_proximity(self, sensor_port: str) -> float:
        max_dist = 200
        result = sensors.get_sensor_value(sensor_port, 0, max_dist, (62,))
        return max_dist if result is None else result


class Robot:
    def __init__(self, vm: VirtualMachineExt):
        self.vm = vm

    def move_start_forward(self) -> None:
        hub.led(LedColor.Green)
        self.vm.start_move_direction("forward")

    async def rotate_radar_async(self, direction: str, position: float) -> None:
        await self.vm.run_motor_to_position("E", direction, position)

    def get_color(self) -> int:
        return self.vm.get_color("C")

    def get_proximity(self) -> float:
        return self.vm.get_proximity("D")

    async def turn_async(self) -> None:
        await self.rotate_radar_async("counterclockwise", 270)
        yield 100
        left_distance = self.get_proximity()

        await self.rotate_radar_async("clockwise", 90)
        yield 100
        right_distance = self.get_proximity()

        hub.led(LedColor.Black)
        await self.rotate_radar_async("shortest", 0)

        await self.vm.move_degrees_async(
            "counterclockwise" if left_distance > right_distance else "clockwise", 155
        )

        self.move_start_forward()

    async def run(self) -> None:
        self.vm.store.move_pair(("A", "B"))
        self.vm.store.move_speed(50)

        await self.rotate_radar_async("shortest", 0)
        self.move_start_forward()

        while True:
            if self.get_color() == LedColor.Azure:
                hub.led(LedColor.Red)
                await self.vm.move_cm_async("back", 10)
                hub.led(LedColor.Black)
                await self.turn_async()

            if self.get_proximity() < 15:
                self.vm.stop_movement()
                hub.led(LedColor.Yellow)
                if self.get_proximity() < 10:
                    await self.vm.move_cm_async("back", 10 * self.get_proximity())
                await self.turn_async()

            yield


async def main(vm: VirtualMachineExt, stack) -> None:
    robot = Robot(vm)
    await robot.run()
    vm.stop()


def setup(rpc, system, stop) -> VirtualMachineExt:
    vm = VirtualMachineExt(rpc, system, stop)
    vm.register_on_start(None, main)
    return vm

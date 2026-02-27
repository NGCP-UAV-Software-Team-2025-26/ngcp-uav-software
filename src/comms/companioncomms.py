#!/usr/bin/env python3
import asyncio
from mavsdk import System

SITL_ADDRESS = "udp://:14540"

async def print_connection(drone: System):
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected to PX4 (SITL)")
            return


async def wait_ready(drone: System):
    print("Waiting for GPS/home health (SITL)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✓ Global position OK + Home position OK")
            return
        await asyncio.sleep(0.5)


async def telemetry_loop(drone: System):
    async def flight_mode_task():
        async for fm in drone.telemetry.flight_mode():
            print(f"[MODE] {fm}")
            await asyncio.sleep(0.2)

    async def position_task():
        async for pos in drone.telemetry.position():
            print(f"[POS] lat={pos.latitude_deg:.6f} lon={pos.longitude_deg:.6f} rel_alt={pos.relative_altitude_m:.1f}m")
            await asyncio.sleep(0.5)

    await asyncio.gather(flight_mode_task(), position_task())


async def rtl_on_enter(drone: System):
    # Simple “ground authority test” from companion:
    # Press Enter → RTL command sent.
    loop = asyncio.get_running_loop()
    print("\nPress ENTER to send RTL (Return-to-Launch). Ctrl+C to quit.\n")
    await loop.run_in_executor(None, input)

    try:
        await drone.action.return_to_launch()
        print("✓ RTL command sent")
    except Exception as e:
        print(f"✗ Failed to send RTL: {e}")


async def main():
    drone = System()
    print(f"Connecting to {SITL_ADDRESS} ...")
    await drone.connect(system_address=SITL_ADDRESS)

    await print_connection(drone)
    await wait_ready(drone)

    # Run telemetry printing + wait for user RTL trigger in parallel
    await asyncio.gather(
        telemetry_loop(drone),
        rtl_on_enter(drone),
    )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting.")


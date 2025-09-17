import asyncio
from mavsdk import System

async def run():
	drone = System()
	await drone.connect(system_address="udpin://0.0.0.0:14540")

	async for state in drone.core.connection_state():
		print(f"Connected: {state.is_connected}")
		if state.is_connected:
			break
	await drone.action.arm()
	await drone.action.takeoff()
	await asyncio.sleep(10)
	await drone.action.land()
	async for in_air in drone.telemetry.in_air():
		if not in_air:
			print("Landed!")
			break
if __name__ == "__main__":
	asyncio.run(run())

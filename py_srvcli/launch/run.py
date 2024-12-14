import asyncio
from launch import LaunchService
from main_launch import generate_launch_description


async def main():
    launch_service = LaunchService()
    launch_description = generate_launch_description()
    launch_service.include_launch_description(launch_description)
    await launch_service.run_async()


if __name__ == '__main__':
    asyncio.run(main())

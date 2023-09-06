from nicegui import ui
import os
import signal
import subprocess


TABS = ["Home",
        "Simulation", 
        "Observers"]

LAUNCH = [
    "ros2 launch multirobot_bringup multirobot_bringup.launch.xml",
    "ros2 run multirobot_formation agent_formation_node",
    "ros2 run multirobot_formation mock_test",
    "ros2 run multirobot_formation resetPose",
    "ros2 run multirobot_formation orchestrator_node",]

OBSERVERS = [
    "ros2 run formation_error_observer main",
    "ros2 run localization_error_observer localization_error_observer 4",
    "ros2 run mileage_observer main 10 4",
    "ros2 run power_usage_observer main 4",]

process = None
task = None


async def read_markdown_file(file_path):
    with open(file_path, "r") as f:
        markdown_content = f.read()
    return markdown_content

# Global variable to store the processes
ros2_processes = []

async def start_ros2_launch(ros_launch_command):
    global ros2_processes

    try:
        # Start the ROS 2 launch process
        process = subprocess.Popen(
            ros_launch_command,
            shell=True,
            cwd=os.path.dirname(os.path.abspath(__file__)),
            preexec_fn=os.setsid  # This is used to create a process group to enable sending signals to the entire group
        )
        ros2_processes.append(process)
        print(f"ROS 2 launch process started: {ros_launch_command}")
    except Exception as e:
        print(f"Error starting ROS 2 launch process: {str(e)}")

async def stop_ros2_launch():
    global ros2_processes

    for process in ros2_processes:
        try:
            # Send the SIGINT signal to the entire process group to gracefully terminate ROS 2 launch
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait()  # Wait for the process to finish
            print(f"ROS 2 launch process terminated.")
        except Exception as e:
            print(f"Error stopping ROS 2 launch process: {str(e)}")

    ros2_processes = []

@ui.page('/')
def view():
    with ui.footer(value=True).style('background-color: #bd94ea') as footer:
        ui.label(
            'SwarmBots @ Fraunhofer IPA 2023').classes('absolute-center items-center')
    with ui.column().classes('w-full items-center'):
        ui.label('SwarmBots Mission Controller').classes(
            'text-h2 mt-10').style('color: #bd94ea')
        # ui.label('ROS 2 Demonstrator 326').classes('text-h4 my-2')
        with ui.tabs().classes('w-1/2 justify-center').props('h2') as tabs:
            for tab in TABS:
                ui.tab(tab).classes('w-1/2 text-center').style('font-size: 20px;')

        with ui.tab_panels(tabs, value=TABS[1]):
            with ui.tab_panel(TABS[0]):
                # with ui.card().classes('mt-10 mb-1 w-96 h-96').style('width: 64vw; position: relative'):
                with ui.row().classes('text-center'):
                    ui.label('')
                    ui.image('assets/swarmbots-graphic.png').classes('mt-10 mb-1 w-96 h-96').style('width: 50vw; position: relative')

            with ui.tab_panel(TABS[1]):
                with ui.card().classes('mt-10 mb-1 w-96 h-56').style('width: 60vw; position: relative'):
                    with ui.row().classes('text-center'):
                        ui.label("Basic functions").classes("text-h6 text-muted")
                        with ui.row():
                            ui.button('Start Simulation', color='#9312fa', on_click=lambda: start_ros2_launch(LAUNCH[0])).style('color: white; font-size: 20px; padding: 20px 20px; border-radius: 50px;').classes("my-10 items-center")
                            ui.button('Start Formation', color='#9312fa', on_click=lambda: start_ros2_launch(LAUNCH[1])).style('color: white; font-size: 20px; padding: 20px 20px; border-radius: 50px;').classes("my-10 items-center")
                            ui.button('Start Orchestration', color='#9312fa', on_click=lambda: start_ros2_launch(LAUNCH[4])).style('color: white; font-size: 20px; padding: 20px 20px; border-radius: 50px;').classes("my-10 items-center")                            
                with ui.card().classes('mt-10 mb-1 w-96 h-56').style('width: 60vw; position: relative'):
                    with ui.row().classes('text-center'):
                        ui.label("Helper functions").classes("text-h6 text-muted")                        
                        with ui.row():
                            ui.button('Test formation', color='#9312fa', on_click=lambda: start_ros2_launch(LAUNCH[2])).style('color: white; font-size: 20px; padding: 20px 20px; border-radius: 50px;').classes("my-10 items-center")
                            ui.button('Reset Robot Poses', color='#9312fa', on_click=lambda: start_ros2_launch(LAUNCH[3])).style('color: white; font-size: 20px; padding: 20px 20px; border-radius: 50px;').classes("my-10 items-center")

                with ui.card().classes('mt-10 mb-1 w-96 h-56').style('width: 60vw; position: relative'):
                    with ui.row().classes('text-center'):
                        ui.label("End testing").classes("text-h6 text-muted")
                        with ui.row():
                            ui.button('Stop Simulation', color='#cb5153', on_click=stop_ros2_launch).style('color: white; font-size: 20px; padding: 20px 20px; border-radius: 50px;').classes("my-10 absolute-center")
                            

            with ui.tab_panel(TABS[2]):
                ui.label("Select the property observer").classes(
                    "text-h4 my-2 text-center").style('color: #9312fa')
                with ui.card().classes('mt-10 mb-1 w-96 h-56').style('width: 60vw; position: relative'):
                    with ui.row().classes('text-center'):
                        ui.label("").classes("text-h6 text-muted")
                        with ui.row():
                            ui.button('Formation', color='#9312fa', on_click=lambda: start_ros2_launch(OBSERVERS[0])).style('color: white; font-size: 20px; padding: 20px 20px; border-radius: 50px;').classes("my-10 justify-center")

                            ui.button('Localization', color='#9312fa', on_click=lambda: start_ros2_launch(OBSERVERS[1])).style('color: white; font-size: 20px; padding: 20px 20px; border-radius: 50px;').classes("my-10 justify-center")

                            ui.button('Mileage', color='#9312fa', on_click=lambda: start_ros2_launch(OBSERVERS[3])).style('color: white; font-size: 20px; padding: 20px 20px; border-radius: 50px;').classes("my-10 justify-center")

                            ui.button('Power usage', color='#9312fa', on_click=lambda: start_ros2_launch(OBSERVERS[2])).style('color: white; font-size: 20px; padding: 20px 20px; border-radius: 50px;').classes("my-10 justify-center")

                with ui.card().classes('mt-10 mb-1 w-96 h-56').style('width: 60vw; position: relative'):
                    with ui.row().classes('text-center'):
                        ui.label("").classes("text-h6 text-muted")
                        with ui.row():
                            ui.button('Stop', color='#cb5153', on_click=stop_ros2_launch).style('color: white; font-size: 20px; padding: 20px 20px; border-radius: 50px;').classes("my-10 absolute-center")

            # with ui.tab_panel(TABS[3]):
            #     ui.label('Documentation').classes('text-h4 my-2 text-center').style('color: #25957b')
            #     markdown_content = read_markdown_file("multirobot_mission_control/static/docs.md")
            #     ui.markdown(markdown_content)




ui.run()



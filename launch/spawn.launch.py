import xacro, os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python import get_package_prefix, get_package_share_directory


def generate_launch_description():

    # Establece los nombres de nuestro paquete y del archivo con el modelo.
    package_name = 'py_gazebo_spawn_example'
    urdf_file_name = 'ball_model.urdf'


    # DESCOMENTAR si el codigo URDF utiliza archivos mesh donde la ruta al
    # archivo de malla es relativa, indicandose de la forma: 
    # <mesh filename ="package://mi_paquete/..../archivo_mesh" />
    # --------------------------------------------------------------------------
    #gazebo_model_path = os.path.join(get_package_prefix(package_name), 'share')


    # DESCOMENTAR si el codigo URDF utiliza archivos mesh donde la ruta al
    # archivo de malla es relativa a la ubicación del archivo setup.py, 
    # indicandose de la forma: <mesh filename ="package://..../archivo_mesh" />
    # --------------------------------------------------------------------------
    #gazebo_model_path = get_package_share_directory(package_name)


    # DESCOMENTAR si el codigo URDF utiliza archivos mesh donde la ruta al
    # archivo de malla se indica de forma relativa.
    # --------------------------------------------------------------------------
    #if 'GAZEBO_MODEL_PATH' in os.environ:
    #    os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + gazebo_model_path
    #else:
    #    os.environ['GAZEBO_MODEL_PATH'] =  gazebo_model_path


    # Obtiene la ruta al archivo que contiene la información de nuestro modelo.
    urdf_file_path = os.path.join(
        get_package_share_directory(package_name), 'urdf', urdf_file_name)


    # Crea y configura el nodo ROBOT_STATE_PUBLISHER, el cual lee el código URDF 
    # del modelo y resuelve la cinemática directa del robot a partir de la 
    # información que proporciona el nodo JOINT_STATE_PUBLISHER_GUI. 
    # Utiliza Command para ejecutar xacro y obtener el código URDF.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}],
        arguments=['--ros-args', '--log-level', 'error'],
        output='screen')
    

    # Crea y configura el nodo SPAWN_ENTITY, que es el que va a cargar y mostrar
    # el modelo en Gazebo.
    node_spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', urdf_file_name,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'],
        output='screen')


    # Incluye la ejecución del archivo launch gazebo.launch.py, que forma parte
    # del paquete gazebo_ros.
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]),
             )


    # Devuelve un objeto LaunchDescription con las acciones a realizar.
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        node_spawn_entity,
    ])

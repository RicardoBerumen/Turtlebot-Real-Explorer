#!/bin/bash
# Validar si se pasó un argumento
if [ -z "$1" ]; then
  echo "Tienes que poner el puerto en linea de comandos "
  exit 1
fi

# Tomar el número del argumento
PUERTO="$1"

# Ejecutar el comando con el puerto especificado
if sudo chmod 777 /dev/ttyUSB"$PUERTO"; then
  # Confirmar la ejecución
  echo "Permisos cambiados para /dev/ttyUSB$PUERTO"
else
  # Confirmar la ejecución
  echo "lo pusiste mal Victor"
  exit 
fi




# source
source install/setup.bash

#build al agente
ros2 run micro_ros_setup build_agent.sh


#run al agente
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB"$PUERTO"

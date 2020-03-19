#!/usr/bin/env bash

#Joao Avelino @ VisLab - Institute for Systems and Robotics (IST-UL) 2018
#Many thanks to Pedro Vicente, Carlos Cardoso and Plinio Moreno
#Tested on Ubuntu 16.04

install_gazebo_yarp_plugins () {

YARP_REPOSITORIES=$1
cd $YARP_REPOSITORIES
git clone https://github.com/robotology/gazebo-yarp-plugins.git
cd gazebo-yarp-plugins && git checkout 3bc9b82d07a0f510666753309be560a426a559dd
mkdir build install && cd build
cmake ../ -DCMAKE_INSTALL_PREFIX=$YARP_REPOSITORIES/install
make -j$(nproc)
make install
echo "export GAZEBO_PLUGIN_PATH=\${GAZEBO_PLUGIN_PATH}:$YARP_REPOSITORIES/install/lib" >> $HOME/.yarp_env
source $HOME/.yarp_env

}

add_variables () {
	    YARP_REPOSITORIES=$1 DIR=$2
	    VIZZY_YARP_DIR="$(echo $DIR | awk -F "scripts" '{print $1}')" 
	    echo "export YARP_ROOT=$YARP_REPOSITORIES/yarp" >> $HOME/.yarp_env
	    echo "export YARP_DIR=\$YARP_ROOT/build" >> $HOME/.yarp_env
	    echo "export YARP_ROBOT_NAME=vizzy" >> $HOME/.yarp_env
	    echo "export ICUB_ROOT=$YARP_REPOSITORIES/icub-main" >> $HOME/.yarp_env
	    echo "export ICUB_DIR=\$ICUB_ROOT/build" >> $HOME/.yarp_env
	    echo "export VIZZY_YARP_ICUB_ROOT=$VIZZY_YARP_DIR" >> $HOME/.yarp_env
	    echo "export VIZZY_YARP_ICUB_DIR=\$VIZZY_YARP_ICUB_ROOT/build" >> $HOME/.yarp_env
	    echo "export ICUBcontrib_DIR=$YARP_REPOSITORIES/icub-contrib-common/build" >> $HOME/.yarp_env
	    echo "export YARP_DATA_DIRS=\$YARP_DIR/share/yarp:\$ICUB_DIR/share/iCub:\$ICUBcontrib_DIR/share/ICUBcontrib:\$VIZZY_YARP_ICUB_DIR/share/yarp" >> $HOME/.yarp_env
	    echo "export YARP_COLORED_OUTPUT=1" >> $HOME/.yarp_env
	    echo "export YARP_FORWARD_LOG_ENABLE=1" >> $HOME/.yarp_env
	    echo "export PATH=\$PATH:\$YARP_DIR/bin:\$ICUB_DIR/bin:\$ICUBcontrib_DIR/bin:\$VIZZY_YARP_ICUB_DIR/bin" >> $HOME/.yarp_env
}


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

printf "[Vizzy]: Hello %s. I'm Vizzy and I'm going to help you to instal YARP and YARP related stuff that I need for simulation. I'm here to make your life easier! :) \n\n\n" "$USER"

YARP_REPOSITORIES=$HOME/yarp_repositories

read -p "[Vizzy]: Where should I install yarp and related stuff? Default: $YARP_REPOSITORIES   " NEW_YARP_REPOSITORIES

if [ ! -z "$NEW_YARP_REPOSITORIES" ]; then
  YARP_REPOSITORIES=$NEW_YARP_REPOSITORIES
  echo "[Vizzy]: I'm going to install to: $YARP_REPOSITORIES"
fi


printf "\n\n[Vizzy]: I will start by checking if a yarp package is installed and if environmental variables are defined which might indicate a previous yarp installation...\n\n"

if [ ! $(dpkg-query -W -f='${Status}' yarp 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
  echo "[Vizzy]: YARP package is installed... do you wish to uninstall it and continue this script or abort this script?"
  options=("Uninstall YARP package" "Quit")
  select opt in "${options[@]}"
  do
    case $opt in
        "Uninstall YARP package")
            echo "[Vizzy]: Uninstalling yarp\n"
	    sudo apt-get remove --purge yarp
	    break
            ;;
        "Quit")
	    echo "[Vizzy]: Oh, ok. Bye..."
	    exit;
            ;;
        *) echo "invalid option $REPLY";;
    esac
  done
else
  printf "[Vizzy]: Good! I found no YARP package installed. Let's continue...\n"
fi

set_vars=0

if [ -z ${YARP_ROOT+x} ]; then 
  echo "YARP_ROOT is unset"; 
else 
  echo "YARP_ROOT is set to '$YARP_ROOT'";
  set_vars=1 
fi

if [ -z ${YARP_DIR+x} ]; then 
  echo "YARP_DIR is unset"; 
else 
  echo "YARP_DIR is set to '$YARP_DIR'";
  set_vars=1
fi

if [ -z ${YARP_ROBOT+x} ]; then 
  echo "YARP_ROBOT is unset"; 
else 
  echo "YARP_ROBOT is set to '$YARP_ROBOT'";
  set_vars=1
fi

if [ -z ${ICUB_ROOT+x} ]; then 
  echo "ICUB_ROOT is unset"; 
else 
  echo "ICUB_ROOT is set to '$ICUB_ROOT'";
  set_vars=1
fi

if [ -z ${ICUB_DIR+x} ]; then 
  echo "ICUB_DIR is unset"; 
else 
  echo "ICUB_DIR is set to '$ICUB_DIR'";
  set_vars=1
fi

if [ -z ${VIZZY_YARP_ICUB_ROOT+x} ]; then 
  echo "VIZZY_YARP_ICUB_ROOT is unset"; 
else 
  echo "VIZZY_YARP_ICUB_ROOT is set to '$VIZZY_YARP_ICUB_ROOT'";
  set_vars=1
fi

if [ -z ${VIZZY_YARP_ICUB_DIR+x} ]; then 
  echo "VIZZY_YARP_ICUB_DIR is unset"; 
else 
  echo "VIZZY_YARP_ICUB_DIR is set to '$VIZZY_YARP_ICUB_DIR'";
  set_vars=1
fi

if [ -z ${ICUBcontrib_DIR+x} ]; then 
  echo "ICUBcontrib_DIR is unset"; 
else 
  echo "ICUBcontrib_DIR is set to '$ICUBcontrib_DIR'";
  set_vars=1
fi

if [ -z ${YARP_DATA_DIRS+x} ]; then 
  echo "YARP_DATA_DIRS is unset"; 
else 
  echo "YARP_DATA_DIRS is set to '$YARP_DATA_DIRS'";
  set_vars=1
fi

if [ -z ${YARP_COLORED_OUTPUT+x} ]; then 
  echo "YARP_COLORED_OUTPUT is unset"; 
else 
  echo "YARP_COLORED_OUTPUT is set to '$YARP_COLORED_OUTPUT'";
  set_vars=1
fi

if [ -z ${YARP_FORWARD_LOG_ENABLE+x} ]; then 
  echo "YARP_FORWARD_LOG_ENABLE is unset"; 
else 
  echo "YARP_FORWARD_LOG_ENABLE '$YARP_ROOT'";
  set_vars=1
fi

if [ -z ${YARP_FOWARD_LOG_ENABLE+x} ]; then 
  echo "YARP_ROOT is unset"; 
else 
  echo "YARP_ROOT is set to '$YARP_ROOT'";
  set_vars=1
fi


if [ $set_vars -gt 0 ]; then
  echo "[Vizzy]: Looks like some of the environment variables are alread set. Please clean up files that might be exporting them first. Tip: .bashrc, .bash_env and .yarp_env are good candidate files..."
  exit;
fi 

printf "[Vizzy]: Let me create the yarp_repositories folder on your home folder and the .yarp_env file\n"


if [ ! -d "$YARP_REPOSITORIES" ]; then
  #Create YARP repositories if it does not exist
  printf "[Vizzy]: yarp_repositories does not exist. Creating it...\n"
  mkdir $YARP_REPOSITORIES
else
  if find $YARP_REPOSITORIES -mindepth 1 -print -quit 2>/dev/null | grep -q .; then
    printf "[Vizzy]: I found a non empty yarp_repositories folder\n"
    
  else
    printf "[Vizzy]: Cool, an empty yarp_repositores already exists\n"
  fi 
fi

if [ ! -f "$HOME/.yarp_env" ]; then
  echo "[Vizzy]: .yarp_env does not exist. Creating it!"
  touch $HOME/.yarp_env
  add_variables $YARP_REPOSITORIES $DIR
else
  echo "[Vizzy]: .yarp_env exists..."
  echo "[Vizzy]: It has the following content:"
  echo "$(cat $HOME/.yarp_env)"
  printf "\n\n [Vizzy]: What should I do?\n\n"

  options=("Add variables" "Delete content and add variables" "Continue without adding variables" "Quit")
  select opt in "${options[@]}"
  do
    case $opt in
	"Add variables")
	    printf "[Vizzy]: Ok! Let's add the variables!\n"
	    add_variables $YARP_REPOSITORIES $DIR
	    break
            ;;
	"Delete content and add variables")
	    rm $HOME/.yarp_env
	    touch $HOME/.yarp_env
	    add_variables $YARP_REPOSITORIES $DIR
	    break
	    ;;
	"Continue without adding variables")
	    printf "[Vizzy]: Ok, you seem confident. Let's continue\n"
	    break
	    ;;
	"Quit")
	    echo "[Vizzy]: Ok... See you later!"
	    exit
    esac
  done
fi
  

check_bashrc=$(grep "$HOME/.yarp_env" $HOME/.bashrc)

if [ -n "$check_bashrc" ]; then echo "[Vizzy]: .bashrc already exports .yarp_env. I do not need to add it."
else
echo "source $HOME/.yarp_env" >> $HOME/.bashrc
fi

source $HOME/.yarp_env

printf "\n\n[Vizzy]: Let me check if all variables are set and loaded:\n\n"
printf "########################################################################\n"
echo "# YARP_ROOT=$YARP_ROOT"
echo "# YARP_DIR=$YARP_DIR"
echo "# YARP_ROBOT_NAME=$YARP_ROBOT_NAME"
echo "# ICUB_ROOT=$ICUB_ROOT"
echo "# ICUB_DIR=$ICUB_DIR"
echo "# VIZZY_YARP_ICUB_ROOT=$VIZZY_YARP_ICUB_ROOT"
echo "# VIZZY_YARP_ICUB_DIR=$VIZZY_YARP_ICUB_DIR"
echo "# ICUBcontrib_DIR=$ICUBcontrib_DIR"
echo "# YARP_DATA_DIRS=$YARP_DATA_DIRS"
echo "# YARP_COLORED_OUTPUT=$YARP_COLORED_OUTPUT"
echo "# YARP_FORWARD_LOG_ENABLE=$YARP_FORWARD_LOG_ENABLE"
echo "##########################################################################\n\n"


printf "\n\n [Vizzy:] All good?\n"

options=("Yes" "No (quit)")
select opt in "${options[@]}"
do
  case $opt in
      "Yes")
        printf "[Vizzy]: Ok! Let's continue!\n"
          break
          ;;
      "No (quit)")
          echo "[Vizzy]: Ok, fix that and rerun this script. See you later (I don't know how to help :[ )!"
          exit
  esac
done

printf "[Vizzy]: Let's install dependencies if they are not installed...\n"

sudo apt-get install -y coinor-libipopt-dev
sudo apt-get install -y libgtkmm-2.4-dev
sudo apt-get install -y git
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y libace-dev
sudo apt-get install -y libgsl-dev
sudo apt-get install -y libedit-dev
sudo apt-get install -y cmake
sudo apt-get install -y cmake-curses-gui
sudo apt-get install -y libsuitesparse-dev
sudo apt-get install -y qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev \
  qtdeclarative5-qtquick2-plugin qtdeclarative5-window-plugin \
  qml-module-qtquick-controls\
  qml-module-qtquick-dialogs\
  qmlscene

echo "[Vizzy]: Now let's install and compile a tested version of YARP"
cd $YARP_REPOSITORIES
git clone https://github.com/robotology/yarp.git && cd yarp
git checkout 8efdca9a8ae994bfdd28f2ad72bcefcdce2eedba
mkdir build && cd build
cmake -DCREATE_GUIS=ON -DCREATE_DEVICE_LIBRARY_MODULES=ON -DCREATE_LIB_MATH=ON -DCREATE_OPTIONAL_CARRIERS=ON -DENABLE_YARPRUN_LOG=ON -DENABLE_yarpcar_bayer=ON -DENABLE_yarpcar_mjpeg=ON -DENABLE_yarpcar_rossrv=ON -DENABLE_yarpcar_tcpros=ON -DENABLE_yarpcar_xmlrpc=ON -DENABLE_yarpmod_serial=ON -DBUILD_SHARED_LIBS=ON ..
make -j$(nproc)

if [ $? -eq 0 ]; then
    printf "\n\n[Vizzy]: I downloaded and compiled yarp 2.3.70.1 (commit: 8efdca9a8ae994bfdd28f2ad72bcefcdce2eedba)\n\n"
else
    printf "[Vizzy]: Oh now, an error... I do not know what to do!"
    exit
fi


echo "[Vizzy]: Now lets install and compile icub-main"
cd $YARP_REPOSITORIES
git clone https://github.com/robotology/icub-main.git && cd icub-main
git checkout 07016c66032031c15bb5741ec88d460ceedc7ec5
mkdir build && cd build
cmake -DENABLE_icubmod_cartesiancontrollerclient=ON -DENABLE_icubmod_cartesiancontrollerserver=ON -DENABLE_icubmod_gazecontrollerclient=ON ..
make -j$(nproc)


if [ $? -eq 0 ]; then
    printf "\n\n[Vizzy]: If no errors occurred then icub-main is compiled :)\n\n"
else
    printf "[Vizzy]: Oh now, an error... I do not know what to do!"
    exit
fi



printf "[Vizzy]: Now let's download and compile icub-contrib-common...\n"

cd $YARP_REPOSITORIES
git clone https://github.com/robotology/icub-contrib-common.git
cd icub-contrib-common
git checkout d870318b5b27bbe9f2e4395c8ca80347d6bfbda5
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$(echo $ICUBcontrib_DIR) ..
make install


if [ $? -eq 0 ]; then
    printf "[Vizzy]: Success!"
else
    printf "[Vizzy]: Oh now, an error... I do not know what to do!"
    exit
fi



printf "\n\n[Vizzy]: Installing and compiling robots-configuration...\n\n"
cd $YARP_REPOSITORIES
git clone https://github.com/robotology/robots-configuration.git
cd robots-configuration
git checkout e2e9807f1438ed8090f96b0d29e1eb6a6f3b9dec
mkdir build && cd build
cmake ../
make install

if [ $? -eq 0 ]; then
    printf "[Vizzy]: Success!"
else
    printf "[Vizzy]: Oh now, an error... I do not know what to do!"
    exit
fi

print "\nNow lets check gazebo-yarp-plugins\n"


cd $YARP_REPOSITORIES
#First check if the GAZEBO_PLUGIN_PATH variable is set

if [ -z ${GAZEBO_PLUGIN_PATH+x} ]; then
  echo "GAZEBO_PLUGIN_PATH is unset";
  install_gazebo_yarp_plugins $YARP_REPOSITORIES
else
  echo "GAZEBO_PLUGIN_PATH '$GAZEBO_PLUGIN_PATH'";
  #If the variable is set we need to check out of it contains the path to gazebo-yarp-plugins with one of the .so's we are expecting to have... It might be set for other plugins!
  FILELIST=($(echo $GAZEBO_PLUGIN_PATH | awk -v RS":" '{print $1}'))
  found=0
  for i in "${FILELIST[@]}"
  do
    if [ -f $i/libgazebo_yarp_worldinterface.so ]; then
        found=1
	break;
    fi
  done
  if [ $found = 0 ]; then
    install_gazebo_yarp_plugins $YARP_REPOSITORIES
  fi
fi


printf "\n\n [Vizzy]: Now lets download the vizzy tactile repository\n\n"

#Get the catkin ws
CATKIN_WS="$(echo $DIR | awk -F "/src/vizzy" '{print $1}')"

cd $CATKIN_WS/src

git clone https://github.com/vislab-tecnico-lisboa/vizzy_tactile_drivers.git
cd $CATKIN_WS
catkin_make
if [ $? -eq 0 ]; then
    printf "\n [Vizzy]: Good! Vizzy tactile repository compiled!\n"
else
    printf "\n [Vizzy]: Oh no... an error :(\n"
    exit
fi


printf "\n [Vizzy]: I'm now going to generate the necessary YARP messages\n"

source $CATKIN_WS/devel/setup.bash

#cd $CATKIN_WS/src/vizzy/vizzy_yarp_icub/src/modules/armGesture/include
#yarpidl_rosmsg --out . Int16
#yarpidl_rosmsg --out . TactSensorArray 

cd $CATKIN_WS/src/vizzy/vizzy_yarp_icub/
mkdir -p build && rm -rf build/* && cd build
cmake -DALLOW_IDL_GENERATION=ON -DCMAKE_CXX_STANDARD=11 ..
make -j$(nproc)

if [ $? -eq 0 ]; then
    printf "\n [Vizzy]: My YARP modules were successfully compiled!\n"
else
    printf "\n [Vizzy]: Oh no... an error :(\n"
    exit
fi

source $HOME/.yarp_env

printf "\n [Vizzy]: The END!\n"


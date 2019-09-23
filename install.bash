#!/bin/bash

YELLOW='\033[0;33m'
BYELLOW='\033[1;33m'
NC='\033[0m'

# Add Aliases / Shortcuts
##################################################################
#alias sr 2>/dev/null >/dev/null && echo "foo is set as an alias"

if [ "$(grep '^alias sr=' ~/.bash* ~/.bash_aliases ~/.profile /etc/bash* /etc/profile)" ]
then 
  echo -e "${BYELLOW}[INFO] ${YELLOW}'sr' is already set as an alias${NC}"
else 
  echo -e "${BYELLOW}[INFO] ${YELLOW}Setting 'sr' as an alias for 'source ~/.bashrc'${NC}"
  echo -e "alias sr='source ~/.bashrc'" >> ~/.bash_aliases
fi

if [ "$(grep '^alias sw=' ~/.bash* ~/.bash_aliases ~/.profile /etc/bash* /etc/profile)" ]
then
  echo -e "${BYELLOW}[INFO] ${YELLOW}'sw' is already set as an alias${NC}"
else
  echo -e "${BYELLOW}[INFO] ${YELLOW}Setting 'sw' as an alias for 'source ./devel/setup.bash'${NC}"
  echo -e "alias sw='source ./devel/setup.bash'" >> ~/.bash_aliases
fi

# Add ROSSERIAL deps
##################################################################
echo -e "${BYELLOW}[INFO] ${YELLOW}Installing rosserial dependencies${NC}"
sudo apt install ros-melodic-rosserial
sudo apt install ros-melodic-rosserial-arduino
sudo apt install ros-melodic-rosserial-msgs
sudo apt install ros-melodic-rosserial-python

# Add Teleop deps
#################################################################
echo -e "${BYELLOW}[INFO] ${YELLOW}Installing teleop-twist dependencies${NC}"
sudo apt install ros-melodic-teleop-twist-joy
sudo apt install ros-melodic-teleop-twist-keyboard

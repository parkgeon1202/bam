# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

from .testbench import Pendulum
from .erob.actuator import ErobActuator
from .dynamixel.actuator import MXActuator, MX28Actuator, MX64V2Actuator, MX106V2Actuator, XHActuator, XL320Actuator, XL330Actuator, XL330CurrentActuator
from .feetech.actuator import STS3215Actuator
from .unitree.actuator import UnitreeGo1Actuator

actuators = {
    # Dynamixel MX series
    "mx64": lambda: MXActuator(Pendulum),
    "mx64v2": lambda: MX64V2Actuator(Pendulum),  # MX-64(2.0), Protocol 2.0 firmware
    "mx106": lambda: MXActuator(Pendulum),
    "mx106v2": lambda: MX106V2Actuator(Pendulum),  # MX-106(2.0), Protocol 2.0 firmware
    "mx28": lambda: MX28Actuator(Pendulum),
    "xh540": lambda: XHActuator(Pendulum),
    # Dynamixel XL series
    "xl320": lambda: XL320Actuator(Pendulum),
    "xl330": lambda: XL330Actuator(Pendulum),
    "xl330i": lambda: XL330CurrentActuator(Pendulum),
    
    # eRob actuators with custom PD controller
    "erob80_100": lambda: ErobActuator(Pendulum, damping=2.0),
    "erob80_50": lambda: ErobActuator(Pendulum, damping=1.0),

    # Feetech STS3215
    "sts3215": lambda: STS3215Actuator(Pendulum),

    # Unitree Go1
    "unitree_go1": lambda: UnitreeGo1Actuator(Pendulum)
}
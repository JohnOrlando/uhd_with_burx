#
# Copyright 2010 Ettus Research LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#


INSTALL(FILES
    #### props headers ###
    codec_props.hpp
    dboard_props.hpp
    device_props.hpp
    dsp_props.hpp
    mboard_props.hpp
    subdev_props.hpp

    #### dboard headers ###
    dboard_base.hpp
    dboard_eeprom.hpp
    dboard_id.hpp
    dboard_iface.hpp
    dboard_manager.hpp

    ### utilities ###
    dsp_utils.hpp
    gps_ctrl.hpp
    mboard_eeprom.hpp
    misc_utils.hpp
    subdev_spec.hpp
    tune_helper.hpp

    ### interfaces ###
    single_usrp.hpp
    multi_usrp.hpp

    DESTINATION ${INCLUDE_DIR}/uhd/usrp
)

#include "bot_firmware/bot_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bot_firmware{

BumperbotInterface::BumperbotInterface()
{
    

}
BumperbotInterface::~BumperbotInterface()
{
    if (arduino_.IsOpen()){
        try{
            arduino_.Close();
        }catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"), "something went wroong when closing the connection at port"<<port_);
        }
    }
}

CallbackReturn BumperbotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info){
    CallbackReturn result= hardware_interface::SystemInterface::on_init(hardware_info);
    if (result!= CallbackReturn::SUCCESS){
        return result;
    }
    try{
        port_ = info_.hardware_parameters.at("port");
    }
    catch( const std::out_of_range &e){
        RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"),"no serial port provided so aborting");
    }
    velocity_commands_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
  // Check 
}

std::vector<hardware_interface::StateInterface> BumperbotInterface::export_state_interfaces(){

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i=0; i<info_.joints.size(); i++){
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name,
        hardware_interface::HW_IF_POSITION,&position_states_.at(i)));

        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name,
        hardware_interface::HW_IF_VELOCITY,&position_states_.at(i)));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BumperbotInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interface;
    for(size_t i=0; i<info_.joints.size(); i++){
        command_interface.emplace_back(hardware_interface::CommandInterface(info_.joints.at(i).name,
        hardware_interface::HW_IF_VELOCITY,&velocity_commands_.at(i)));
    }
    return command_interface;
}

CallbackReturn BumperbotInterface::on_activate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),"starting robot hardware..");
    velocity_commands_={0.0, 0.0 ,0.0, 0.0};
    position_states_={0.0, 0.0 ,0.0, 0.0};
    velocity_states_={0.0, 0.0 ,0.0, 0.0};

    try{
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    } catch(...){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),"something went wrong while interacting with the port");
    }
    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),"hardware started and ready to take commands");
    return CallbackReturn::SUCCESS;
}
CallbackReturn BumperbotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),"hardware started and ready to take commands");
    if (arduino_.IsOpen()){
        try{
            arduino_.Close();
        } catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),"something went wrong while CLOSING THE port");
            return CallbackReturn::FAILURE;
        }
    }
}

hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period){

    
}
        virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
   

}

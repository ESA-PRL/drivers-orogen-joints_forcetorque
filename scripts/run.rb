#! /usr/bin/env ruby

require 'orocos'
require 'vizkit'
include Orocos

Orocos.initialize
Orocos.run 'joints_forcetorque::Task' => 'joints_forcetorque' do

	driver = Orocos.name_service.get 'joints_forcetorque'

    driver.num_sensors = 1
    driver.torque_ratio = 1000000
    driver.force_ratio = 1000000
	driver.etherCAT_aliases = 1
	driver.name_sensors = 'test sensor'

	driver.configure
	driver.start
	
	Vizkit.display driver.ft_sensors

	loop do
		sleep 0.01
	end
   
end


/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "ecrt.h"

using namespace joints_forcetorque;

	
Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
  
    /** Get the configurations **/
	num_sensors = _num_sensors.get();
	torque_ratio = _torque_ratio.get();
	force_ratio = _force_ratio.get();
	aliasID = _etherCAT_aliases.get(); // TODO use this in the code
	sensor_name = _name_sensors.get();
	
	std::cout << num_sensors << std::endl;
	std::cout << sensor_name << std::endl;
    
	/* EtherCAT */
	master = NULL;
	domain1 = NULL;
	domain1_pd = NULL;
	sc_ATI_MINI45 = NULL;


	// TODO master parameter will mostly be 0, but could hypotetically change, must be configured
    master = ecrt_request_master(0);
    if (!master)
    {
		std::cerr << "Error opening master device '" << master << "'" << std::endl;
        return false;
	}

	/* Create a domain to read from (probably) can make several domains 
	 * for the same master, depending on what we want to read, 
	 * check ecrt_domain_reg_pdo_entry_list function */
    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
		std::cerr << "Error setting up domain master device '" << domain1 << "'" << std::endl;
        return false;
	}

    if (!(sc_ATI_MINI45 = ecrt_master_slave_config(
                    master, ATI_MINI45_alias_pos, ATI_MINI45))) 
	{
		std::cerr << "Failed to get slave configuration for ATI_MINI45 '" << stderr << "'" << std::endl;
        return false;
    }
    
    /************************************* NEW **********************************/
    if (!(sc_JUNCTION0 = ecrt_master_slave_config(
                    master, 0,0, 0x00000002,0x23a84052))) 
	{
		std::cerr << "Failed to get slave configuration for ATI_MINI45 '" << stderr << "'" << std::endl;
        return false;
    }
    /************************************* NEW **********************************/


    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_ATI_MINI45, EC_END, ATI_MINI45_syncs)) {
		std::cerr << "Failed to configure ATI_MINI45 PDOs '" << stderr << "'" << std::endl;
        return false;
    }
	
    if (ecrt_domain_reg_pdo_entry_list(domain1, ATI_MINI45_domain_regs)) {
		std::cerr << "PDO entry registration failed '" << stderr << "'" << std::endl;
        return false;
    }

	/**************************************/
	//ecrt_slave_config_dc(sc_JUNCTION0, 0x0700, 10000000, 4400000, 0, 0);
	//ret = ecrt_master_select_reference_clock(master, sc_JUNCTION0);
    if (ecrt_master_select_reference_clock(master, sc_JUNCTION0) < 0) {
		std::cerr << "PDO entry registration failed '" << stderr << "'" << std::endl;
        return false;
    }

	// This is not needed here. If it is needed it should probably go in the start hook
    //setup sdo
    //if (!(sdo_forceUnits = ecrt_slave_config_create_sdo_request(sc_ATI_MINI45, 0x7010, 0x1, 4))) {
    //   fprintf(stderr, "Failed to create SDO request.\n");
    //   return -1;
    //}
    //ecrt_sdo_request_timeout(sdo_forceUnits, 500); // ms
	
    return true;
}
bool Task::startHook()
{
	
	sync_ref_counter = 0;
	
    if (! TaskBase::startHook())
        return false;
        
    printf("Activating master...\n");
    if (int ret = ecrt_master_activate(master))
    {
		std::cerr << "Failed activating master '" << ret << "'" << std::endl;
        return false;
	}

    if (!(domain1_pd = ecrt_domain_data(domain1))) 
    {
		std::cerr << "Failed contacting slave '" << domain1_pd << "'" << std::endl;
        return false;
    }   
     
    return true;
}
void Task::updateHook(){

	TaskBase::updateHook();
	

	/* receive process data */
	ecrt_master_receive(master);
	ecrt_domain_process(domain1);


	/*printf("ATI MINI45 count %d  \n", EC_READ_U32(domain1_pd + off_ATI_MINI45_counter));
	printf("ATI MINI45 status %x  \n", EC_READ_U32(domain1_pd + off_ATI_MINI45_status));
	printf("ATI MINI45 control %lld \n", (long long)EC_READ_U64(domain1_pd + off_ATI_MINI45_control));	
	printf("ATI MINI45 gauges %d \t %d \t %d \t %d \t %d \t %d \n \n",
	EC_READ_U32(domain1_pd + off_ATI_MINI45_FX),
	EC_READ_U32(domain1_pd + off_ATI_MINI45_FY),
	EC_READ_U32(domain1_pd + off_ATI_MINI45_FZ),
	EC_READ_U32(domain1_pd + off_ATI_MINI45_MX),
	EC_READ_U32(domain1_pd + off_ATI_MINI45_MY),
	EC_READ_U32(domain1_pd + off_ATI_MINI45_MZ)
	);*/

	sensor1.time = base::Time::now();
	sensor1.force[0] = (int)(EC_READ_U32(domain1_pd + off_ATI_MINI45_FX))/force_ratio;
	sensor1.force[1] = (int)(EC_READ_U32(domain1_pd + off_ATI_MINI45_FY))/force_ratio;
	sensor1.force[2] = (int)(EC_READ_U32(domain1_pd + off_ATI_MINI45_FZ))/force_ratio;
	
	sensor1.torque[0] = (int)(EC_READ_U32(domain1_pd + off_ATI_MINI45_MX))/torque_ratio;
	sensor1.torque[1] = (int)(EC_READ_U32(domain1_pd + off_ATI_MINI45_MY))/torque_ratio;
	sensor1.torque[2] = (int)(EC_READ_U32(domain1_pd + off_ATI_MINI45_MZ))/torque_ratio;

	// write application time to master
	ecrt_master_application_time(master, sensor1.time.microseconds*1000);

	if (sync_ref_counter) {
		sync_ref_counter--;
	} else {
		sync_ref_counter = 1; // sync every cycle
		ecrt_master_sync_reference_clock(master);
	}
	ecrt_master_sync_slave_clocks(master);	

	/* send process data */
	ecrt_domain_queue(domain1);
	ecrt_master_send(master);
	
	_ft_sensors.write(sensor1);
	

}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    
    ecrt_release_master(master);
}

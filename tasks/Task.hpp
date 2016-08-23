/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef JOINTS_FORCETORQUE_TASK_HPP
#define JOINTS_FORCETORQUE_TASK_HPP

#include "joints_forcetorque/TaskBase.hpp"
#include "ecrt.h"

#define ATI_MINI45_alias_pos    1,0
// vendor and product id
#define ATI_MINI45  0x00000732, 0x26483052

namespace joints_forcetorque {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the ethercat namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','ethercat::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

	/***************************/
	/** Output Port Variables  **/
	/***************************/
	
	base::samples::Wrench sensor1;   // value to be sent in the output port
	
	/***************************/
	/** Config Port Variables  **/
	/***************************/
	
	int num_sensors;
	double torque_ratio;
	double force_ratio;
	int aliasID;
	std::string sensor_name;
	//std::vector<int> aliasID;
	//std::vector<std::string> sensor_name;
       
	/************************************************************************************************************************************************************************/
	/* EtherCAT */
	ec_master_t *master;
	ec_domain_t *domain1;
	uint8_t *domain1_pd; /* process data */
	ec_slave_config_t *sc_ATI_MINI45;
	ec_slave_config_t *sc_JUNCTION0;
	unsigned int sync_ref_counter;



	/* Offsets for PDO entries */
	unsigned int off_ATI_MINI45_FX;
	unsigned int off_ATI_MINI45_FY;
	unsigned int off_ATI_MINI45_FZ;
	unsigned int off_ATI_MINI45_MX;
	unsigned int off_ATI_MINI45_MY;
	unsigned int off_ATI_MINI45_MZ; 
	unsigned int off_ATI_MINI45_status;
	unsigned int off_ATI_MINI45_counter;
	unsigned int off_ATI_MINI45_control;
	
	 ec_pdo_entry_reg_t ATI_MINI45_domain_regs[10] = {
		{ATI_MINI45_alias_pos, ATI_MINI45, 0x6000, 1, &off_ATI_MINI45_FX},
		{ATI_MINI45_alias_pos, ATI_MINI45, 0x6000, 2, &off_ATI_MINI45_FY},
		{ATI_MINI45_alias_pos, ATI_MINI45, 0x6000, 3, &off_ATI_MINI45_FZ},
		{ATI_MINI45_alias_pos, ATI_MINI45, 0x6000, 4, &off_ATI_MINI45_MX},
		{ATI_MINI45_alias_pos, ATI_MINI45, 0x6000, 5, &off_ATI_MINI45_MY},
		{ATI_MINI45_alias_pos, ATI_MINI45, 0x6000, 6, &off_ATI_MINI45_MZ},    
		{ATI_MINI45_alias_pos, ATI_MINI45, 0x6010, 0, &off_ATI_MINI45_status},
		{ATI_MINI45_alias_pos, ATI_MINI45, 0x6020, 0, &off_ATI_MINI45_counter},
		{ATI_MINI45_alias_pos, ATI_MINI45, 0x7010, 1, &off_ATI_MINI45_control},
		{}
	};

	/* Obtained by having just one sensor on the bus and calling "ethercat cstruct" in a terminal */
	ec_pdo_entry_info_t ATI_MINI45_pdo_entries[10] = {
		{0x7010, 0x01, 32}, /* Control 1 */
		{0x7010, 0x02, 32}, /* Control 2 */
		{0x6000, 0x01, 32}, /* Fx/Gage0 */
		{0x6000, 0x02, 32}, /* Fy/Gage1 */
		{0x6000, 0x03, 32}, /* Fz/Gage2 */
		{0x6000, 0x04, 32}, /* Tx/Gage3 */
		{0x6000, 0x05, 32}, /* Ty/Gage3 */
		{0x6000, 0x06, 32}, /* Tz/Gage3 */
		{0x6010, 0x00, 32}, /* SubIndex 000 */
		{0x6020, 0x00, 32}, /* SubIndex 000 */
	};

	ec_pdo_info_t ATI_MINI45_pdos[2] = {
		{0x1601, 2, ATI_MINI45_pdo_entries + 0}, /* DO RxPDO-Map */
		{0x1a00, 8, ATI_MINI45_pdo_entries + 2}, /* DI TxPDO-Map */
	};

	ec_sync_info_t ATI_MINI45_syncs[5] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, ATI_MINI45_pdos + 0, EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 1, ATI_MINI45_pdos + 1, EC_WD_DISABLE},
		{0xff}
	};


	/*ec_pdo_entry_reg_t ATI_MINI45_domain_regs[10];

	ec_pdo_entry_info_t ATI_MINI45_pdo_entries[10];

	ec_pdo_info_t ATI_MINI45_pdos[2];

	ec_sync_info_t ATI_MINI45_syncs[5];*/

	/************************************************************************************************************************************************************************/

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "joints_forcetorque::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif


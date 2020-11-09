// Hardware interface for the CM730 board
// File:    CM730.cpp
// Authors: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>
//          Sebastian Schueller <schuell1@cs.uni-bonn.de>
// Comment: This file is suitable for CM730 firmware versions of 0x81 and above (NimbRo-OP specific).

// Includes - Local
#include <opencr/CM730.h>

// Includes - System
#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <sstream>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/poll.h>
#include <termios.h>

// Includes - ROS
#include <ros/console.h>

// Defines - Configuration
#define CM730_COMMS_DEBUG             0 // Non-zero => Display TX and RX packets
#define CM730_COMMS_DEBUG_TX          0 // Non-zero => Display TX packets
#define BULKREAD_DETAILED_TIMESTAMPS  0 // Non-zero => Display bulk read timing information
#define DISPLAY_NONFAIL_TIMES         0 // Non-zero => Display bulk read times if no failure as well
#define DISPLAY_BULKREAD_TIME         0 // Non-zero => Display a message with the bulk read times
#define DISPLAY_FAIL_COUNTS           0 // Non-zero => Display the local servo fail counts

// Defines
#define COMMS_READ_TIMEOUT  10  		// Read timeout in ms (it's ok if this is longer than a single robotcontrol time step, but it shouldn't be too long so that the read in the next cycle has a chance without a cycle being missed)
#define FULL_BR_PERIOD      5.0         // Maximum time to send repeat bulk read instructions instead of the full bulk read Tx packet

///////////////// for Protocol 2.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8

// Namespaces
using namespace opencr;

// Constants
const char* CM730::PATH = "/dev/cm730";
#define PROTOCOL_VERSION	2.0
#define BAUDRATE 			1000000

// Helper functions
inline void getTimeLimit(struct timespec* out);
inline int makeWord(unsigned char* pos);
inline int makeByteSigned(unsigned char* pos);
inline int makeWordSigned(unsigned char* pos);
void dump(const char* prefix, const uint8_t* data, uint8_t len);

//
// CM730 class
//

// Constructor
CM730::CM730(const std::string& resourcePath, const std::string& configParamPath, boost::shared_ptr<const DynamixelBase> servos)
 : m_servos(servos)
 , m_PM(0, resourcePath) // Used to plot events only, hence zero size
 , m_fd(-1)
 , m_lastFailedID(0)
 , m_fullBRPacket(true)
 , m_failCount() // Initialise all fail counts to zero
 , m_isSuspended(false)
 , m_wasSuspended(false)
 , m_unsuspendTime(0, 0)
 , m_gotCM730Data(false)
 , m_lastSeenDynPow(DYNPOW_OFF)
 // Open a serial connection to the CM730
 , m_portHandler(dynamixel::PortHandler::getPortHandler(PATH))
 , m_packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION))
 , m_groupBulkRead(m_portHandler, m_packetHandler)
{
	// Initialise the bulk read TX packet
	memset(&m_TxBulkRead, 0, sizeof(m_TxBulkRead));
	updateTxBRPacket(std::vector<int>());
}

// Destructor
CM730::~CM730()
{
}

//
// Connection functions
//

// Connect to the CM730
int CM730::connect()
{
	// Declare variables
	struct termios config;
	struct serial_struct serinfo;

	// Open port
  	if (!m_portHandler->openPort())
  	{
	    perror("[CM730] Could not open serial connection");
		return -1;
	}

	// Set port baudrate
  	if (!m_portHandler->setBaudRate(BAUDRATE))
  	{
    	perror("[CM730] Could not set baudrate");
    	m_portHandler->closePort();
		return -1;
  	}

  	m_portHandler->setPacketTimeout((uint16_t) COMMS_READ_TIMEOUT);

  	m_portHandler->clearPort();

	// Return that the connection was successful
	return 0;
}

// Send a ping command
int CM730::ping(int id, struct timespec* abstime)
{
	// For protocol 1.0 only
	// We send the packet:  [0xFF] [0xFF] [ID] [0x02] [PING] [Checksum]
	// And hope to receive: [0xFF] [0xFF] [ID] [0x02] [Status] [Checksum]

	// Declare variables
	unsigned char idbyte;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  	uint8_t dxl_error = 0;                          // Dynamixel error

  	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

	// Check the passed ID
	idbyte = (unsigned char) id;
	if(idbyte >= ID_BROADCAST)
		return RET_BAD_PARAM;

	// Send ping command and retrieve 
	dxl_comm_result = m_packetHandler->ping(m_portHandler, idbyte, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("CM730 ping() comm error: %s", m_packetHandler->getTxRxResult(dxl_comm_result));
		return dxl_comm_result;
	}
	else if(dxl_error != 0)
	{
		ROS_ERROR("CM730 ping() dxl error: %s", m_packetHandler->getRxPacketError(dxl_error));
		return dxl_error;
	}
	
	return COMM_SUCCESS;
}

// Suspend the CM730 communications for a given amount of time
void CM730::suspend(double suspendTime)
{
	// Disable bytes from reaching the dynamixels for the duration of the suspend (disables PC --> DXL forwarding, i.e. TX to the DXL from the CM730)
	if(m_lastSeenDynPow != DYNPOW_OFF) // Note: We need to be careful here that we don't enable dynamixel power if it's not supposed to be on!
		setDynamixelPower(DYNPOW_ON_NODXLTX);

	// Suspend the communications as required
	if(suspendTime >= 0.0)
		m_unsuspendTime = ros::Time::now() + ros::Duration(suspendTime); // Time-limited comms suspension...
	else
		m_unsuspendTime.fromNSec(0); // Open-ended comms suspension...
	m_isSuspended = true;
	m_wasSuspended = true;

	// Plot an event for the communications suspension
	std::ostringstream sstream;
	sstream.precision(3);
	sstream << "CM730 Susp " << std::fixed << suspendTime << "s";
	m_PM.clear();
	m_PM.plotEvent(sstream.str());
	m_PM.publish();
}

// Unsuspend the CM730 communications
void CM730::unsuspend()
{
	// Unsuspend the communications as required
	m_isSuspended = false;
	m_wasSuspended = false;
	
	// Allow bytes to reach the dynamixels again (enables PC --> DXL forwarding, i.e. TX to the DXL from the CM730)
	if(m_lastSeenDynPow != DYNPOW_OFF) // Note: We need to be careful here that we don't enable dynamixel power if it's not supposed to be on!
		setDynamixelPower(DYNPOW_ON);
}

//
// Send instruction function
//

// Send a particular instruction (with no parameters)
int CM730::sendInstruction(int id, int inst)
{
	// We send the packet: [0xFF] [0xFF] [ID] [0x02] [INST] [Checksum]


	int result                 = COMM_TX_FAIL;

	uint8_t txpacket[10]        = {0};

	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

	if (id >= BROADCAST_ID)
	  return COMM_NOT_AVAILABLE;

	txpacket[PKT_ID]            = id;
	txpacket[PKT_LENGTH_L]      = 3;
	txpacket[PKT_LENGTH_H]      = 0;
	txpacket[PKT_INSTRUCTION]   = (unsigned char) inst;

	result = m_packetHandler->txPacket(m_portHandler, txpacket);

	return result;
}

//
// Read functions
//

// Read one register byte from a device
int CM730::readByte(int id, int address, int* value, struct timespec* abstime)
{
	// We send the packet:  [0xFF] [0xFF] [ID] [0x04] [READ] [Addr] [0x01] [Checksum]
	// And hope to receive: [0xFF] [0xFF] [ID] [0x03] [Status] [Data] [Checksum]

	// Declare variables
	unsigned char rxp=0;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  	uint8_t dxl_error = 0;

  	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

	// Wait to receive a return packet
	dxl_comm_result = m_packetHandler->read1ByteTxRx(m_portHandler, 
													(uint8_t) id, 
													(uint16_t)address, 
													&rxp, 
													&dxl_error);

	if (dxl_error != 0)
	{
		ROS_ERROR("CM730 readByte() dxl error: %s", m_packetHandler->getRxPacketError(dxl_error));
		return (int)dxl_error;
	}
	else if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("CM730 readByte() comm error: %s", m_packetHandler->getTxRxResult(dxl_comm_result));
		return dxl_comm_result;
	}

	*value = (int)rxp;

	// Return success
	return COMM_SUCCESS;
}

// Read one register word from a device
int CM730::readWord(int id, int address, int* value, struct timespec* abstime)
{
	// We send the packet:  [0xFF] [0xFF] [ID] [0x04] [READ] [Addr] [0x02] [Checksum]
	// And hope to receive: [0xFF] [0xFF] [ID] [0x04] [Status] [Data1] [Data2] [Checksum]


	// Declare variables
	uint16_t rxp=0;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  	uint8_t dxl_error = 0;

  	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

	// Wait to receive a return packet
	dxl_comm_result = m_packetHandler->read2ByteTxRx(m_portHandler, 
													(uint8_t) id, 
													(uint16_t) address, 
													&rxp, 
													&dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("CM730 readWord() comm error: %s", m_packetHandler->getTxRxResult(dxl_comm_result));
		return dxl_comm_result;
	}
	else if (dxl_error != 0)
	{
		ROS_ERROR("CM730 readWord() dxl error: %s", m_packetHandler->getRxPacketError(dxl_error));
		return (int)dxl_error;
	}

	*value = (int)rxp;

	// Return success
	return COMM_SUCCESS;
}

// Read register data from a device
int CM730::readData(int id, int address, void* data, size_t size, struct timespec* abstime)
{
	// We send the packet:  [0xFF] [0xFF] [ID] [0x04] [READ] [Addr] [N] [Checksum]
	// And hope to receive: [0xFF] [0xFF] [ID] [N+2] [Status] [Data1] ... [DataN] [Checksum]


	// Declare variables
	unsigned char rxp[MAX_RX_BYTES] = {0};
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  	uint8_t dxl_error = 0;

  	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

	// Wait to receive a return packet
	dxl_comm_result = m_packetHandler->readTxRx(m_portHandler, 
												(unsigned char) id, 
												address,  
												size,
												rxp, 
												&dxl_error);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("CM730 readData() comm error: %s id: %d addr: %d size: %d", 
			m_packetHandler->getTxRxResult(dxl_comm_result),
			id,
			address,
			size);
		return dxl_comm_result;
	}
	else if (dxl_error != 0)
	{
		ROS_ERROR("CM730 readData() dxl error: %s", m_packetHandler->getRxPacketError(dxl_error));
		return (int)dxl_error;
	}

	// Return the read data
	memcpy(data, rxp, size);

	// Return success
	return COMM_SUCCESS;
}

//
// Bulk read functions
//

// Perform a read of the CM730 only
int CM730::readCM730(BRBoard* cm730Data)
{
	// Reset the last failed ID variable
	m_lastFailedID = 0;
	
	// Reset whether we received CM730 data
	m_gotCM730Data = false;
	
	// Start timing if desired
#if DISPLAY_BULKREAD_TIME
	ros::Time tstart = ros::Time::now();
#endif

	// Declare variables
	unsigned char rxp[MAX_RX_BYTES] = {0};

	// Calculate the desired absolute timeout time
	struct timespec timeout_time;
	getTimeLimit(&timeout_time);
	
	// Retrieve the register range to read from the CM730 from the bulk read packet (Note: Assumes the CM730 is the first device in the bulk read packet!)
	int id = ID_CM730;
	int len = m_groupBulkRead.GetDataLength(ID_CM730);
	int address = m_groupBulkRead.GetAddress(ID_CM730);
	
	// Save the requested data headers
	cm730Data->id = id;
	cm730Data->length = len;
	cm730Data->startAddress = address;
	
	// Read the required register range from the CM730 (
	pauseSuspend();
	int rxError = readData(id, address, rxp, len, &timeout_time);
	resumeSuspend();
	
	// Handle the case where the read is unsuccessful
	if(rxError != COMM_SUCCESS)
	{
		m_lastFailedID = id;
		if((id >= 0) && (id < MAX_DEVICES))
		{
			m_failCount[id]++;
#if DISPLAY_FAIL_COUNTS
			ROS_ERROR("ID %d failed: Total %d times now", id, m_failCount[id]);
#endif
		}
		else
			ROS_ERROR("Invalid ID %d failed", id);
#if DISPLAY_BULKREAD_TIME
		ROS_WARN("Total CM730 read time: %lfs [FAIL]", (ros::Time::now() - tstart).toSec());
#endif
		return rxError;
	}
	
	// Display the CM730 read time
#if DISPLAY_BULKREAD_TIME
	ROS_WARN("Total CM730 read time: %lfs", (ros::Time::now() - tstart).toSec());
#endif
	
	// Parse the received read response packet
	if(!parseCM730Data(rxp, cm730Data))
		return RET_RX_CORRUPT;

	// Return success
	return COMM_SUCCESS;
}

// Perform a bulk read
int CM730::bulkRead(std::vector<BRData>* servoData, BRBoard* cm730Data)
{
	int dxl_comm_result = COMM_TX_FAIL;
	uint8_t dxl_error = 0;
	bool dxl_getdata_result = false;

	// Reset the last failed ID variable
	m_lastFailedID = 0;
	
	// Reset whether we received CM730 data
	m_gotCM730Data = false;

	// Save the start ROS time of the call to this function
#if BULKREAD_DETAILED_TIMESTAMPS || DISPLAY_BULKREAD_TIME
	std::vector<ros::Time> ts;
	ts.push_back(ros::Time::now());
#endif

	// Declare variables
	int i;

	// Calculate the desired absolute timeout time
	struct timespec timeout_time;
	getTimeLimit(&timeout_time);

	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

	// Send the bulk read Tx packet
	ros::Time preBulkTx = ros::Time::now();
	if((preBulkTx - m_lastFullBRPacket).toSec() > FULL_BR_PERIOD)
		m_fullBRPacket = true;

	dxl_comm_result = m_groupBulkRead.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
    	ROS_ERROR("CM730 bulkRead() comm error: %s", m_packetHandler->getTxRxResult(dxl_comm_result));
    	return dxl_comm_result;
    }
    m_fullBRPacket = false;
	m_lastFullBRPacket = preBulkTx;

	// Save the current ROS time
#if BULKREAD_DETAILED_TIMESTAMPS
	ts.push_back(ros::Time::now());
#endif

	// Give the OS some time (the assumption is that after 3.7ms the servos won't be done responding yet, and the buffer inside the read thread won't be overflowing yet with responses)
	// usleep(3700); // This is an approximate equal of the time it takes before the CM730/servo responses come streaming in...

	// Calculate the number of requests that were made in the bulk read (i.e. the number of queried devices)
	std::vector<unsigned char> deviceIds = m_groupBulkRead.GetListId();
	int numberOfRequests = deviceIds.size();

	// Save the bulk read request data headers (these are used when parsing the bulk read return packets)
	for(i = 0; i < numberOfRequests; i++)
	{
		// Retrieve the requested address, length and ID of the next listed device
		int id   = deviceIds[i];
		uint16_t len  = m_groupBulkRead.GetDataLength(id);
		uint16_t addr = m_groupBulkRead.GetAddress(id);

		// Save the request data headers into the appropriate struct
		if(id == ID_CM730)
		{
			cm730Data->id = id;
			cm730Data->length = len;
			cm730Data->startAddress = addr;
		}
		else
		{
			BRData* servoInfo = &servoData->at(id - 1); // The servo data is stored by ID in a zero-based array, hence subtract 1 here to shift to zero-based
			servoInfo->id = id;
			servoInfo->length = len;
			servoInfo->startAddress = addr;
		}
	}

	dxl_comm_result = m_groupBulkRead.rxPacket();

	// Receive the returned status packets
	for(i = 0; i < numberOfRequests; i++)
	{
		// Retrieve the requested length and ID
		int id  = deviceIds[i];
		uint16_t len  = m_groupBulkRead.GetDataLength(id);
		uint16_t addr = m_groupBulkRead.GetAddress(id);

		dxl_getdata_result = m_groupBulkRead.isAvailable(id, addr, len);
		m_groupBulkRead.getError(id, &dxl_error);
		
		// Handle case where packet fails to arrive or is otherwise corrupt
		if(dxl_getdata_result != true)
		{
			// If no servo responded at all then make sure the next bulk read is sent in full, just to be safe
			if(i <= 1) // Note: i == 0 should always correspond to the CM730, so if the first-queried servo just failed to respond we have i == 3...
				m_fullBRPacket = true;

			// Save the current ROS time
#if BULKREAD_DETAILED_TIMESTAMPS
			ts.push_back(ros::Time::now());
#endif

			// Log an error message
#if BULKREAD_DETAILED_TIMESTAMPS
			ROS_ERROR("Could not read answer for ID %d: Error %s", id, m_packetHandler->getRxPacketError(dxl_error));
#endif

			// Display timing information of the bulk read
#if DISPLAY_BULKREAD_TIME
			ROS_WARN("Total bulk read time: %lfs [FAIL]", (ros::Time::now() - ts[0]).toSec());
#endif

			// Save which ID failed (although the real cause could at times be another servo)
			m_lastFailedID = id;
			if((id >= 0) && (id < MAX_DEVICES))
			{
				m_failCount[id]++;
#if DISPLAY_FAIL_COUNTS
				ROS_ERROR("ID %d failed: Total %d times now", id, m_failCount[id]);
#endif
			}
			else
				ROS_ERROR("Invalid ID %d failed", id);

			// Display the recorded ROS times
#if BULKREAD_DETAILED_TIMESTAMPS
			if(ts.back() - ts[0] >= ros::Duration((double) COMMS_READ_TIMEOUT * 1e-3))
			{
				ROS_WARN("Total time: %lfs", (ts.back() - ts[0]).toSec());
				for(size_t j = 1; j < ts.size(); j++)
					ROS_WARN("Delta: %lfs (%lfs since start)", (ts[j] - ts[j-1]).toSec(), (ts[j] - ts[0]).toSec());
			}
#endif

			// Return the error that occurred
			return dxl_error;
		}

		// Save the current ROS time
#if BULKREAD_DETAILED_TIMESTAMPS
		ts.push_back(ros::Time::now());
#endif

		// Parse the received bulk read response packet
		parseBRPacket(id, servoData, cm730Data);

		// Save the current ROS time
#if BULKREAD_DETAILED_TIMESTAMPS
		ts.push_back(ros::Time::now());
#endif
	}

	// Display timing information if required
#if DISPLAY_BULKREAD_TIME
	ROS_WARN("Total bulk read time: %lfs", (ros::Time::now() - ts[0]).toSec());
#endif
#if BULKREAD_DETAILED_TIMESTAMPS && DISPLAY_NONFAIL_TIMES
	ROS_WARN("Total time: %lfs", (ts.back() - ts[0]).toSec());
	for(size_t j = 1; j < ts.size(); j++)
		ROS_WARN("Delta: %lfs (%lfs since start)", (ts[j] - ts[j-1]).toSec(), (ts[j] - ts[0]).toSec());
#endif

	// Return success
	return COMM_SUCCESS;
}

// Rebuild the Tx packet that is sent for each bulk read based on the passed list of servos
int CM730::updateTxBRPacket(const std::vector<int>& servos)
{
	// For protocol 1.0
	// We want the packet: [0xFF] [0xFF] [0xFE] [3*N+3] [0x92] [0x00] [L1] [ID1] [Addr1] ... [LN] [IDN] [AddrN] [Checksum]
	// For protocol 2.0 please check the robotis official website

	bool dxl_addparam_result = false;

	m_groupBulkRead.clearParam();

	// Add the CM730 as the first device in the bulk read (important!)
	dxl_addparam_result = m_groupBulkRead.addParam(ID_CM730, READ_CM730_ADDRESS, READ_CM730_LENGTH);
	if (dxl_addparam_result != true)
	{
	  	ROS_ERROR("[ID:%03d] grouBulkRead addparam failed", ID_CM730);
	   	return 0;
	}

	// Add the servos to the bulk read packet
	const unsigned char servoReadLength = m_servos->readLength();
	const unsigned char servoReadAddress = m_servos->readAddress();
	for(size_t i = 0; i < servos.size(); i++)
	{
		// Add parameter storage for the servo
	  	dxl_addparam_result = m_groupBulkRead.addParam(servos[i], servoReadAddress, servoReadLength);
	  	if (dxl_addparam_result != true)
	  	{
	    	ROS_ERROR("[ID:%03d] grouBulkRead addparam failed", servos[i]);
	    	return 0;
	  	}
	}

	// We have modified the m_TxBulkRead packet, so we need to force the next bulk read to send the full bulk read Tx packet
	m_fullBRPacket = true;

	// Return success
	return RET_SUCCESS;
}

//
// Write functions
//

// Write one register byte to a device
int CM730::writeByte(int id, int address, int value)
{
	// For protocol 1.0
	// We send the packet: [0xFF] [0xFF] [ID] [0x04] [WRITE] [Addr] [Data] [Checksum]

	int dxl_comm_result = COMM_TX_FAIL;
	uint8_t dxl_error = 0;

	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

	// Wait to receive a return packet
	// dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, 
	// 												(uint8_t) id, 
	// 												(uint16_t)address, 
	// 												(uint8_t) value, 
	// 												&dxl_error);
	dxl_comm_result = m_packetHandler->write1ByteTxOnly(m_portHandler, 
													(uint8_t) id, 
													(uint16_t)address, 
													(uint8_t) value);
	

	if (dxl_error != 0)
	{
		ROS_ERROR("CM730 writeByte() dxl error: %s", m_packetHandler->getRxPacketError(dxl_error));
		return (int)dxl_error;
	}
	else if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("CM730 writeByte() comm error: %s", m_packetHandler->getTxRxResult(dxl_comm_result));
		return dxl_comm_result;
	}

	m_portHandler->clearPort();

	// Return success
	return COMM_SUCCESS;
}

// Write one register word to a device
int CM730::writeWord(int id, int address, int value)
{
	// We send the packet: [0xFF] [0xFF] [ID] [0x05] [WRITE] [Addr] [Data1] [Data2] [Checksum]

	int dxl_comm_result = COMM_TX_FAIL;
	uint8_t dxl_error = 0;

	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

	// Wait to receive a return packet
	// dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, 
	// 												(uint8_t) id, 
	// 												(uint16_t)address, 
	// 												value, 
	// 												&dxl_error);
	dxl_comm_result = m_packetHandler->write2ByteTxOnly(m_portHandler, 
													(uint8_t) id, 
													(uint16_t)address, 
													value);

	if (dxl_error != 0)
	{
		ROS_ERROR("CM730 writeWord() dxl error: %s", m_packetHandler->getRxPacketError(dxl_error));
		return (int)dxl_error;
	}
	else if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("CM730 writeWord() comm error: %s", m_packetHandler->getTxRxResult(dxl_comm_result));
		return dxl_comm_result;
	}

	m_portHandler->clearPort();

	// Return success
	return COMM_SUCCESS;
}

// Write register data to a device
int CM730::writeData(int id, int address, void* data, size_t size)
{
	// We send the packet: [0xFF] [0xFF] [ID] [N+3] [WRITE] [Addr] [Data1] ... [DataN] [Checksum]

	int dxl_comm_result = COMM_TX_FAIL;
	uint8_t dxl_error = 0;

	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

	// Wait to receive a return packet
	// dxl_comm_result = m_packetHandler->writeTxRx(m_portHandler, 
	// 												(uint8_t) id, 
	// 												(uint16_t)address, 
	// 												size, 
	// 												(uint8_t *) data, 
	// 												&dxl_error);
	dxl_comm_result = m_packetHandler->writeTxRx(m_portHandler, 
													(uint8_t) id, 
													(uint16_t)address, 
													size, 
													(uint8_t *) data);

	if (dxl_error != 0)
	{
		ROS_ERROR("CM730 writeData() dxl error: %s", m_packetHandler->getRxPacketError(dxl_error));
		return (int)dxl_error;
	}
	else if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("CM730 writeData() comm error: %s", m_packetHandler->getTxRxResult(dxl_comm_result));
		return dxl_comm_result;
	}

	m_portHandler->clearPort();

	// Return success
	return COMM_SUCCESS;
}

// Perform a sync write
// address:      The starting address of the write on each device
// numDataBytes: The number of bytes to write for each device (i.e. results in a write to the register range "address --> address + numDataBytes - 1")
// numDevices:   The number of devices to write to (the data for exactly this number of devices should be stored in the data parameter)
// data:         A back to back byte array of IDs and corresponding data (Required array length = (numDataBytes + 1) * numDevices)
int CM730::syncWrite(int address, size_t numDataBytes, size_t numDevices, const uint8_t * data)
{
	// We send the packet:
	//     [0xFF] [0xFF] [ID] [(L+1)*N+4] [SYNC_WRITE] [Addr] [L] [[data]] [Checksum]
	// where L is numDataBytes and N is numDevices.
	// [[data]] is expected to be a back to back array of:
	//     [ID] [Data1] ... [DataL]
	// where L is numDataBytes.

	// Declare variables
	int dxl_comm_result = COMM_TX_FAIL;

	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

    uint16_t param_length = (numDataBytes + 1) * numDevices;
    uint8_t copiedData[param_length];
    memcpy(copiedData, data, param_length);
    dxl_comm_result = m_packetHandler->syncWriteTxOnly(m_portHandler, (uint16_t) address, numDataBytes, copiedData, param_length);
    if (dxl_comm_result != COMM_SUCCESS)
    {
    	ROS_ERROR("CM730 syncWriteTxOnly() comm error: %s", m_packetHandler->getTxRxResult(dxl_comm_result));
    	return dxl_comm_result;
    }

	// Send the write packet and return
	return COMM_SUCCESS;
}

// Set the dyamixel power state of the CM730
int CM730::setDynamixelPower(int value)
{
	// Error checking
	if(value < DYNPOW_MIN || value > DYNPOW_MAX)
	{
		ROS_ERROR("Attempted to write bad value %d to the CM730 dynamixel power register!", value);
		return RET_BAD_PARAM;
	}

	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

	// Write the appropriate register
	int dxl_comm_result = writeByte(ID_CM730, P_DYNAMIXEL_POWER, value);
	if (dxl_comm_result != COMM_SUCCESS)
		ROS_ERROR("Failed to write %d to the CM730 dynamixel power register! (Error: %d)", value, dxl_comm_result);

	// Plot an event for the change in dynamixel power
	m_PM.clear();
	if(value == DYNPOW_OFF)
		m_PM.plotEvent("DXL Power Off");
	else if(value == DYNPOW_ON)
		m_PM.plotEvent("DXL Power On");
	else
		m_PM.plotEvent("DXL Power Custom");
	m_PM.publish();

	// Return value
	return dxl_comm_result;
}

// Get the dynamixel power state of the CM730
int CM730::getDynamixelPower(CM730::DynPowState& state, struct timespec* abstime)
{
	// Don't do anything if the communications are suspended
	if(isSuspended()) return COMM_TX_FAIL;

	// Read the appropriate register
	int value = 0;
	int ret = readByte(ID_CM730, P_DYNAMIXEL_POWER, &value, abstime);

	// Return unknown if the read failed
	if(ret != COMM_SUCCESS)
	{
		state = DYNPOW_UNKNOWN;
		return ret;
	}

	// Transcribe the read dynamixel power state
	if(value >= DYNPOW_MIN && value <= DYNPOW_MAX)
		state = (DynPowState) value;
	else
		state = DYNPOW_UNKNOWN;

	// Return success
	return COMM_SUCCESS;
}

// Get the button state of the CM730
int CM730::getButtonState(int& value, struct timespec* abstime)
{
	// Read the appropriate register
	value = 0;
	int ret = readByte(ID_CM730, P_BUTTON, &value, abstime);
	if(ret != RET_SUCCESS)
		value = 0;
	return ret;
}

// Write to the CM730 that it should beep with a particular frequency and duration
int CM730::beep(float duration, int freq)
{
	// Convert the required duration into ticks
	int durTick = (int)(10.0*duration + 0.5);
	if(durTick < 1)
		durTick = 1;
	else if(durTick > 50)
		durTick = 50;

	// Coerce the required frequency
	if(freq < 0)
		freq = 0;
	else if(freq > 51)
		freq = 51;

	// Construct the data bytes to write
	static const std::size_t numBytes = 2;
	uint8_t data[numBytes] = {(uint8_t) durTick, (uint8_t) freq};

	// Write the beep command to the CM730 and return whether it was successful
	return writeData(ID_CM730, P_BUZZER_PLAY_LENGTH, data, numBytes);
}

// Write to the CM730 that it should play a particular preprogrammed sound
int CM730::sound(int musicIndex)
{
	// Coerce the required music index
	if(musicIndex < 0)
		musicIndex = 0;
	else if(musicIndex > 25)
		musicIndex = 25;

	// Construct the data bytes to write
	static const std::size_t numBytes = 2;
	uint8_t data[numBytes] = {0xFF, (uint8_t) musicIndex};

	// Write the sound command to the CM730 and return whether it was successful
	return writeData(ID_CM730, P_BUZZER_PLAY_LENGTH, data, numBytes);
}

//
// Communications functions
//

// Transmit a packet to the CM730
int CM730::txPacket(unsigned char* txp)
{
	int dxl_comm_result = COMM_TX_FAIL;

	// Don't do anything if the communications are suspended
	if(isSuspended()) return RET_TX_FAIL;

	// Display the packet being sent
#if CM730_COMMS_DEBUG || CM730_COMMS_DEBUG_TX
	dump("TX", txp, length);
#endif

	// Write the packet to the serial port
	dxl_comm_result = m_packetHandler->txPacket(m_portHandler, txp);

	if (dxl_comm_result != COMM_SUCCESS)
	{
		ROS_ERROR("CM730 txPacket() write error: %s", m_packetHandler->getTxRxResult(dxl_comm_result));
		return dxl_comm_result;
	}

	return COMM_SUCCESS;
}

// Receive a packet from the CM730
int CM730::rxPacket(unsigned char* rxp, int size, struct timespec* abstime)
{
	// Don't do anything if the communications are suspended
	if(isSuspended()) return RET_TX_FAIL;

	// Declare variables
	int readErr, readSize = 0;

	// Calculate the absolute timeout time if one wasn't provided
	struct timespec timeout_time;
	if(!abstime)
	{
		getTimeLimit(&timeout_time);
		abstime = &timeout_time;
	}

	// Read until the appropriate number of bytes are received or an error occurs
	while(1)
	{
		// Wait to receive at least as many bytes as we are still missing from our packet
		readErr = m_readThread.read(rxp + readSize, size - readSize, abstime); // Note: This should be the only location that retrieves Rx buffer data from the read thread!

		// Abort if we have timed out
		if(readErr == -ETIMEDOUT)
			return RET_RX_TIMEOUT;

		// Abort if an error occurred
		if(readErr < 0)
			return RET_RX_FAIL;

		// Display the received data
#if CM730_COMMS_DEBUG
		dump("RX       ", rxp + readSize, readErr);
#endif

		// Make note of the number of bytes we received 
		readSize += readErr;

		// Synchronise the start of rxp with the start of a packet
		syncRxPacket(rxp, &readSize);

		// See if we have read enough bytes
		if(readSize >= size) // Note: It should be impossible for us to ever have read more than size bytes, so theoretically readSize == size would suffice here...
		{
			// Display the received packet
#if CM730_COMMS_DEBUG
			dump("RX packet", rxp, readSize);
#endif

			// Retrieve the packet length
			int packetSize = rxp[DP_LENGTH] + 4;
			
			// Discard packets that are too short
			if(packetSize < size) // The packet at the beginning of rxp is too short, but we have still received the right number of bytes, so there must be another packet, which might be ours! Because we already know that we have received more bytes it is unlikely that we cause timeouts with this heuristic that otherwise would not have been.
			{
				memset(rxp, 0, packetSize);
				syncRxPacket(rxp, &readSize);
				if(readSize < size) continue;
			}
			
			// Check that the packet is of the length that we were expecting
			if(packetSize != size)
			{
				ROS_ERROR_THROTTLE(0.1, "Wrong Rx packet length: %d (expected %d)", packetSize, (int) size);
				return RET_RX_CORRUPT;
			}

			// Check that the checksum of the packet is correct
			int txChecksum = checksum(rxp);
			if(txChecksum != rxp[DP_LENGTH + rxp[DP_LENGTH]]) // This is safe as at this point we must have rxp[DP_LENGTH] + 4 == size
			{
				ROS_ERROR_THROTTLE(0.1, "Wrong Rx packet checksum: %d (expected %d)", (int) rxp[DP_LENGTH + rxp[DP_LENGTH]], txChecksum);
				return RET_RX_CORRUPT;
			}

			// Return success
			return RET_SUCCESS;
		}
	}
}

// Flush the Rx buffer data (reset to a clean slate)
void CM730::flushPort()
{
	// Tell the read thread to flush the Rx buffers
	m_readThread.flush();
}

//
// Packet helper functions
//

// Parse a bulk read return packet for the information it contains
bool CM730::parseBRPacket(uint8_t id, std::vector<BRData>* servoData, BRBoard* cm730Data)
{
	// Parse the bulk read return packet based on whether it's the CM730 or a servo
	if(id == ID_CM730)
	{
		// Parse the CM730 data
		uint16_t len  = cm730Data->length;
		int start_address = cm730Data->startAddress;
		std::vector<uint8_t> data = m_groupBulkRead.getDatas(id, start_address, len);
		return parseCM730Data(&data[0], cm730Data);
	}
	else
	{
		// Check the servo ID
		if(id > servoData->size())
		{
			ROS_WARN_THROTTLE(0.3, "Servo bulk read return packet comes from ID %d, which is outside our servo data array size (%d)", (int) id, (int) servoData->size());
			return false;
		}

		// Retrieve the servo information struct based on the ID
		BRData* servoInfo = &servoData->at(id - 1); // The servo data is stored by ID in a zero-based array, hence subtract 1 here to shift to zero-based

		// Save the returned data
		uint16_t len  = m_groupBulkRead.GetDataLength(id);
		uint16_t addr = m_groupBulkRead.GetAddress(id);
		servoInfo->position = m_groupBulkRead.getData(id, addr, len);
		
		// Return that we got some data
		return true;
	}
}

// Parse the data of a CM730 packet (data is assumed to point to the first of size bytes of payload data)
bool CM730::parseCM730Data(uint8_t* data, BRBoard* cm730Data)
{
	uint8_t id = cm730Data->id;
	uint16_t len  = cm730Data->length;
	int start_address = cm730Data->startAddress;

	// Calculate the register data address offset
	int offset = -start_address;

	// Save the returned data
	cm730Data->power    = data[offset + P_DYNAMIXEL_POWER];
	cm730Data->ledPanel = data[offset + P_LED_PANEL];
	cm730Data->rgbled5  = makeWord(&data[offset + P_RGBLED_L]);
	cm730Data->rgbled6  = makeWord(&data[offset + P_RGBLED_L]);
	cm730Data->button   = data[offset + P_BUTTON];
	cm730Data->voltage  = data[offset + P_BATTERY_VOLTAGE];
	cm730Data->gyroX    = makeWordSigned(&data[offset + P_GYRO_X_L]);
	cm730Data->gyroY    = makeWordSigned(&data[offset + P_GYRO_Y_L]);
	cm730Data->gyroZ    = makeWordSigned(&data[offset + P_GYRO_Z_L]);
	cm730Data->accX     = makeWordSigned(&data[offset + P_ACC_X_L]);
	cm730Data->accY     = makeWordSigned(&data[offset + P_ACC_Y_L]);
	cm730Data->accZ     = makeWordSigned(&data[offset + P_ACC_Z_L]);
	//cm730Data->magX     = makeWordSigned(&data[offset + P_MAG_X_L]);
	//cm730Data->magY     = makeWordSigned(&data[offset + P_MAG_Y_L]);
	//cm730Data->magZ     = makeWordSigned(&data[offset + P_MAG_Z_L]);
	//cm730Data->temp     = makeByteSigned(&data[offset + P_TEMPERATURE]);
	
	// Save the dynamixel power state for our own purposes
	m_lastSeenDynPow = cm730Data->power;
	
	// Indicate that we received CM730 data
	m_gotCM730Data = true;
	
	// Return that we got some data
	return true;
}

// Synchronise the start of a packet with the start of the passed Rx data buffer (returns the number of bytes that are discarded in the process)
int CM730::syncRxPacket(unsigned char* rxp, int* readSize)
{
	// If we don't have at least 3 bytes then we can't sync anything
	if(*readSize < 3)
		return 0;

	// If the start of our received data is already the start of a packet then all is good
	if(rxp[0] == 0xFF && rxp[1] == 0xFF && rxp[2] != 0xFF)
		return 0;

	// Declare variables
	bool Found = false;
	int i, origSize = *readSize;

	// Search for the first occurrence of a packet header [0xFF] [0xFF] [non-0xFF]
	for(i = 1; i < (*readSize - 2); i++)
	{
		if(rxp[i] == 0xFF && rxp[i+1] == 0xFF && rxp[i+2] != 0xFF)
		{
			Found = true;
			break;
		}
	}

	// First found packet header starts at the zero-based index i => Remove the first i bytes of the Rx buffer
	if(Found)
	{
		size_t copySize = *readSize - i;
		memmove(rxp, rxp + i, copySize);
		*readSize = copySize;
		return i;
	}

	// Check if the last 0 to 2 bytes are a possible incomplete packet header
	if(rxp[*readSize-1] != 0xFF)
	{
		*readSize = 0;
		return origSize;
	}
	else if(rxp[*readSize-2] != 0xFF)
	{
		rxp[0] = 0xFF;
		*readSize = 1;
		return origSize - 1;
	}
	else
	{
		rxp[0] = 0xFF;
		rxp[1] = 0xFF;
		*readSize = 2;
		return origSize - 2;
	}
}

// Calculate the checksum of a packet (assumes that the packet is valid in terms of the length parameter!)
unsigned char CM730::checksum(unsigned char* packet)
{
	// Declare variables
	unsigned char checksum = 0x00;

	// Sum up the bytes of the package
	for(int i = DP_ID; i < packet[DP_LENGTH] + 3; i++)
		checksum += packet[i];

	// Calculate the 1's complement of the sum => This is now the true checksum
	return (~checksum);
}

//
// Helper functions
//

// Calculate a timeout time exactly COMMS_READ_TIMEOUT (ns) into the future based on the current time
inline void getTimeLimit(struct timespec* out)
{
	clock_gettime(CLOCK_MONOTONIC, out);
	out->tv_sec += (out->tv_nsec + COMMS_READ_TIMEOUT) / 1000000000LL;
	out->tv_nsec = (out->tv_nsec + COMMS_READ_TIMEOUT) % 1000000000LL;
}

// Make a word out of two consecutive bytes (pos[0] = Low byte, pos[1] = High byte)
inline int makeWord(unsigned char* pos)
{
	return (uint16_t)(((int) pos[1] << 8) | pos[0]);
}

// Make a signed byte out of an unsigned one
inline int makeByteSigned(unsigned char* pos)
{
	return ((int8_t) *pos);
}

// Make a signed word out of two consecutive bytes (pos[0] = Low byte, pos[1] = High byte)
inline int makeWordSigned(unsigned char* pos)
{
	return (int16_t)(((int) pos[1] << 8) |  pos[0]);
}

// Dump the contents of a byte array to the screen
void dump(const char* prefix, const uint8_t* data, uint8_t len)
{
	printf("%s:", prefix);
	for(int i = 0; i < len; i++)
		printf(" %02X", data[i]);
	printf("\n");
}
// EOF

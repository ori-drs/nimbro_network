// UDP receiver node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

#include <map>
#include <vector>
#include <string>

#include <ros/publisher.h>

#include <topic_tools/shape_shifter.h>

#include <boost/circular_buffer.hpp>

#include "udp_packet.h"

namespace sb_udp
{

struct Message
{
	Message(uint16_t id)
	 : id(id)
	 , size(0)
	 , valid(true)
	{}

	uint32_t getLength() const
	{ return size; }

	uint8_t* getData()
	{ return payload.data(); }

	uint16_t id;
	UDPFirstPacket::Header header;
	std::vector<uint8_t> payload;
	size_t size;
	std::vector<bool> msgs;
	bool valid;
};

struct TopicData
{
	ros::Publisher publisher;
	topic_tools::ShapeShifter shapeShifter;

	uint32_t md5[4];
	std::string md5_str;
	std::string msg_def;
};

class UDPReceiver
{
public:
	UDPReceiver();
	~UDPReceiver();

	void run();
private:
	typedef std::map<std::string, TopicData> TopicMap;
	typedef boost::circular_buffer<Message> MessageBuffer;

	int m_fd;
	MessageBuffer m_incompleteMessages;
	TopicMap m_topics;

	ros::NodeHandle m_nh;
};

}

#endif

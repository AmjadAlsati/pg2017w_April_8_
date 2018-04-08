//////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, CCS Labs
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////////
#include <thread>

#include "cooperative_driving/disable_ros_warnings_pre.h"
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include "cooperative_driving/disable_ros_warnings_post.h"

#include "cooperative_driving/disable_ros_warnings_pre.h"
#include "cooperative_driving/pubscriber.h"
#include "cooperative_driving_networking/BroadcastMessage.h"
#include "cooperative_driving_networking/CooperativeAwarenessMessage.h"
#include "cooperative_driving_networking/EmergencyBrake.h"
#include "cooperative_driving_networking/Platooning.h"
#include "cooperative_driving_networking/StringStamped.h"
#include "cooperative_driving_networking/Token.h"
#include "cooperative_driving/disable_ros_warnings_post.h"
#include "cooperative_driving_msgs/Command.h"

#include "cooperative_driving/networking/wifi_packet.h"

extern "C" {
#include "cooperative_driving/networking/message_types.h"
#include "cooperative_driving/networking/transceiver.h"
#include "cooperative_driving/networking/util.h"
}

using cooperative_driving::Pubscriber;

namespace cooperative_driving
{
const char *MsgTypeStrings[] = { "unknown", "token", "emergency_brake", "cam", "platooning" , "command"};
/** Auxiliary function to convert a mac address given as an array of unsigned char
 * to an std::string.
 *
 * \param[in]  mac      Array of size 6
 * \param[out] mac_str  String representation of mac address *
 */
void mac_to_string(unsigned char *mac, std::string &mac_str)
{
  char mac_chr[18];  // 6 Bytes * 2 (one for each digit) + 5 colons + 1 for NULL byte
  sprintf(mac_chr, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  mac_chr[17] = '\0';
  mac_str = mac_chr;
}

/** Wifi monitoring and packet injection.
 *
 * Instances of this class are able to build and inject packets into the current
 * Wifi channel. They are also capable of monitoring the network and reacting to
 * the received frames accordingly. For that, the interface must be set to
 * "monitoring" mode. If this does not happen automatically when plugging in the WiFi dongles,
 * this needs to be done by running the provided scripts. If the
 * network interface is not set properly, ros will terminitate with an
 * error.
 *
 * The class registers a service named `broadcast_message` which can be triggered
 * by ROS nodes. The message format is as follows:
 *
 * ```
 * string msg
 * ---
 * ```
 *
 * Thus, it takes only a string which is to be broadcast and provides no response
 * (for now). This service can be used for quick testing and debugging.
 *
 * Using the Pubscriber class, WiFiMonitor registers pubscribers for different topics (see code of WiFiMonitor::WiFiMonitor).
 *
 * Apart from services and topics, WiFiMonitor also provides a callback function
 * (WiFiMonitor::send_cb) that can be triggered by a ROS timer. This is however
 * only meant for debugging purpose. It also starts the WiFiMonitor#receiver_thread_ .
 *
 * Once a packet is received, WiFiMonitor parses and deserializes it and publishes relevant content
 * on the relevant topics. ROS nodes can subscribe to this topic and
 * react to the received messages accordingly.
 *
 * WiFiMonitor does not process each and every packet received on the channel. It first
 * filters the source's mac address according to the trusted mac addresses
 * configured in config/wifi_dongles.yaml.
 */
class WiFiMonitor
{
public:
  /** Constructor.
   *
   * Sets up network interface (calling WiFiMonitor::init_interface and open_device()) and parses mac address filter
   * by calling WiFiMonitor::parse_dongle_config.
   * Additionally it registers the service `broadcast_message` and registers pubscribers for different topics.
   *
   * \param [in] name Node's name
   * 
   * \todo Registering of the pubscribers should be done in a separate method rather than in the initializer list.
   */
  WiFiMonitor(const std::string &name)
    : name_(name)
    , string_pub_(Pubscriber<cooperative_driving_networking::StringStamped>(
          nh_, "/networking/broadcast_message", 10, 10,
          [&](const boost::shared_ptr<cooperative_driving_networking::StringStamped const> &msg_ptr) {
            this->broadcast_message(msg_ptr);
          }))
    , token_pub_(Pubscriber<cooperative_driving_networking::Token>(
          nh_, "/networking/token", 10, 10,
          [&](const boost::shared_ptr<cooperative_driving_networking::Token const> &msg_ptr) {
            this->handle_message(msg_ptr, TOKEN);
          }))
    , emergency_brake_pub_(Pubscriber<cooperative_driving_networking::EmergencyBrake>(
          nh_, "/networking/emergency_brake", 10, 10,
          [&](const boost::shared_ptr<cooperative_driving_networking::EmergencyBrake const> &msg_ptr) {
            this->handle_message(msg_ptr, EMERGENCY_BRAKE);
          }))
    , cam_pub_(Pubscriber<cooperative_driving_networking::CooperativeAwarenessMessage>(
          nh_, "/networking/cam", 10, 10,
          [&](const boost::shared_ptr<cooperative_driving_networking::CooperativeAwarenessMessage const> &msg_ptr) {
            this->handle_message(msg_ptr, CAM);
          }))
    , platooning_message_pub_(Pubscriber<cooperative_driving_networking::Platooning>(
          nh_, "/networking/platooning", 10, 10,
          [&](const boost::shared_ptr<cooperative_driving_networking::Platooning const> &msg_ptr) {
            this->handle_message(msg_ptr, PLATOONING);
          }))
    , command_pub_(Pubscriber<cooperative_driving_msgs::Command>(
          nh_, "/networking/command", 10, 10,
          [&](const boost::shared_ptr<cooperative_driving_msgs::Command const> &msg_ptr) {
            this->handle_message(msg_ptr, COMMAND);
          }))
  {
    message_.reset(new WiFiPacket());

    last_received_.seqno_ = 0;
    last_received_.mac_ = "";

    parse_dongle_config();
    init_interface();

    device_ = (device_t *)open_device(&tx_options_);

    // Comment in the following to activate send timer. Useful for debugging.
    // send_timer_ = nh_.createTimer(ros::Rate(1), &Sender::send_cb, this);
    receiver_thread_ = new std::thread(&WiFiMonitor::receive, this);
    service_ = nh_.advertiseService("broadcast_message", &WiFiMonitor::broadcast_message, this);
  }

  /** Destructor.
   *
   */
  ~WiFiMonitor()
  {
    close_device(device_);
    delete receiver_thread_;
  }

private:
  /** Handle message received by one of the pubscribers.
   *
   * The message was received on one of the pubscribers initialized in WiFiMonitor::WiFiMonitor.
   * It is then serialized and saved in WiFiMonitor::message_. Finally, WiFiMonitor::send is invoked.
   *
   * \param[in]  message  The message received by one of the pubscribers.
   * \param[in]  type     The message type.
   *
   * \tparam     T        Should be of a supported ROS message type.
   */
  template <class T>
  void handle_message(const boost::shared_ptr<T const> &message, MessageType type)
  {
    ROS_DEBUG_STREAM("Handle message: \n" << *message << " Type: " << type);

    ros::SerializedMessage ser_message = ros::serialization::serializeMessage(*message);

    // Comment in to test deserialization
    //deserialize(ser_message.message_start, ser_message.num_bytes);

    message_->set_payload(ser_message.message_start, ser_message.num_bytes);
    message_->get_header()->set_type(type);

    send();
  }

  /** Parse mac address filter.
   *
   * The filter is configured in file config/wifi_dongles.yaml.
   * WiFiMonitor will only accept frames received from addresses contained in this config file.
   */
  void parse_dongle_config()
  {
    XmlRpc::XmlRpcValue dongle_list;

    if (nh_.getParam("wifi_dongles", dongle_list))
    {
      for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = dongle_list.begin(); it != dongle_list.end(); ++it)
      {
        std::string dongle_name(it->first);
        std::string param_path = "wifi_dongles/" + dongle_name;

        std::vector<std::string> params;
        if (nh_.getParam(param_path, params))
        {
          dongles_.push_back(params.at(0));
        }
      }
    }
  }

  /** Initialize network interface.
   *
   * Sets different options (e.g. network interface name to "mon0") and obtains own mac address.
   * \see WiFiMonitor::init_mac_address.
   */
  void init_interface()
  {
    init_mac_address();

    tMK2MACAddr src_mac = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // Will be overriden by firmware
    tMK2MACAddr dst_mac = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };  // Broadcast
    tx_options_.TxCHOpts.ChannelNumber = 1;
    tx_options_.TxCHOpts.Priority = MK2_PRIO_0;
    tx_options_.TxCHOpts.Service = MK2_QOS_NOACK;
    tx_options_.TxCHOpts.pMCS = MK2MCS_R12QAM16;
    tx_options_.TxCHOpts.TxPower = 20;
    tx_options_.TxCHOpts.pTxAntenna = MK2_TXANT_DEFAULT;
    tx_options_.TxCHOpts.Bandwidth = MK2BW_20MHz;
    memcpy(tx_options_.TxCHOpts.DestAddr, dst_mac, sizeof tx_options_.TxCHOpts.DestAddr);
    tx_options_.TxCHOpts.EtherType = 0x88b5;
    memcpy(tx_options_.SrcAddr, src_mac, sizeof tx_options_.SrcAddr);
    tx_options_.pInterfaceName = (char *)"mon0";
  }

  /** Obtain mac adress from Wifi dongle and save in WiFiMonitor::mac_address_.
   * \see        mac_to_string
   */
  void init_mac_address()
  {
    unsigned char macc[6];
    get_my_mac(macc);
    mac_to_string(macc, mac_address_);
  }

  /** Callback method, called when service is triggered.
   *
   * Extract message that shall be sent save it in WiFiMonitor::message_. Finally, call WiFiMonitor::send.
   *
   * Message format:
   *
   * ```
   * string msg
   * ---
   * ```
   *
   * \param[in]  request    Service requests containing the interesting data
   *                        according to the above format.
   * \param[out] response  Data that shall be propagated to the triggering node can
   *                       be stored in this parameter (currenty not used)
   *
   * \return     Always true. Callback functions for service request must return a bool.
   * 
   * \todo       Implement more sensible return behaviour.
   */
  bool broadcast_message(cooperative_driving_networking::BroadcastMessage::Request &request,
                         cooperative_driving_networking::BroadcastMessage::Response & /*response*/)
  {
    ROS_DEBUG_STREAM("Service request: " << request.msg);

    message_.reset(new WiFiPacket(request.msg));

    send();

    return true;
  }

  /** Callback method, called if message is received on topic
   *
   * Extract message that shall be sent save it in WiFiMonitor::message_. Finally, call WiFiMonitor::send.
   *
   * \param[in]  message  Structure containing the message string.
   */
  void broadcast_message(const cooperative_driving_networking::StringStamped::ConstPtr &message)
  {
    ROS_DEBUG_STREAM("Listener: Detected message on topic broadcast_message: " << message->data);

    message_.reset(new WiFiPacket(message->data));
    
    send();
  }

  /** Callback method that can be used by ROS timers.
   *
   * \param[in]  timerEvent not used
   */
  void send_cb(const ros::TimerEvent & /*timerEvent*/)
  {
    message_.reset(new WiFiPacket("Test Broadcast"));
    
    send();
  }

  /** Passes message string to send_frame().
   *
   * Before the message is sent, the header is initialized.
   * It contains a sequence id (starting at 1) and the message type
   * (defined in MessageTypes).
   * 
   * Afterwards, the same message is sent a certain number of times, specified by
   * WiFiMonitor::number_of_redundant_packets_.
   */
  void send()
  {
    static long seqno = 1;

    message_->get_header()->set_sequence_number(seqno++);

    ROS_INFO_STREAM("Injecting packet of type " << MsgTypeStrings[message_->get_header()->get_type() + 1]);
    ROS_DEBUG_STREAM("Content: '" << message_->get_header()->as_string() << "' + " << message_->get_payload_length() << " bytes of payload");

    size_t message_length;
    void* packed_message = (void*)message_->serialize(message_length);

    for (size_t i = 0; i < number_of_redundant_packets_; ++i)
    {
      send_frame(device_, packed_message, message_length, tx_options_.TxCHOpts.TxPower);

      // Then sleep for 1/number_of_redundant_packets_ seconds
      // (i.e. sending all packets will take not more than one second)
      ros::Duration(1 / number_of_redundant_packets_).sleep();
    }
  }

  /** Method that is run by WiFiMonitor::receiver_thread_
   *
   * Blocks until a frame on the channel is available and processes it accordingly.
   * Frames whose sender's mac address match one of the configured trusted
   * addresses are processed further. The packet is then parsed, its payload extracted
   * (a serialized ros message) and then deserialized (by WiFiMonitor::deserialize).
   *
   * Frames from not trusted senders or frames that have already been received are discarded.
   */
  void receive()
  {
    do
    {
      uint8_t rcv_payload[2000];
      memset(rcv_payload, 0, 2000);

      struct frame_info_t frame_info;

      int n = receive_frame(device_, rcv_payload, 2000, &frame_info, 1);
      if (n > 0)
      {
        std::string sender_mac;
        mac_to_string(frame_info.src_address, sender_mac);

        if (!own_frame(sender_mac) && sent_by_trusted_sender(sender_mac))
        {
          ROS_DEBUG_STREAM("Captured packet from source " << sender_mac);

          // Make buffer more readable for output
          // char *readable = (char *)malloc(n + 1);
          // get_readable_buffer(rcv_payload, n, readable);
          // ROS_DEBUG("Payload: %s", readable);

          WiFiPacket rcv_packet(rcv_payload, n);

          if (already_received(rcv_packet.get_header()->get_sequence_number(), sender_mac))
            continue;  // Discard packet

          MessageType type = rcv_packet.get_header()->get_type();

          deserialize(rcv_packet.get_payload(), rcv_packet.get_payload_length(), type);
        }
      }
    } while (nh_.ok());
  }

  /** Deserialize received ros message.
   *
   * Before sending, ros messages were serialized in method WiFiMonitor::handle_message.
   * Upon receival of such a packet, this function is called to deserialize
   * the packet's payload (i.e. the serialized ros message).
   *
   * After deserialization, the message is published on the appropriate pubscriber.
   *
   * \param[in]  data  The data string that needs to be deserialized (i.e. the received packet's payload).
   * \param[in]  len   The length of the data string.
   * \param[in]  type  The type of the received message.
   * 
   *  \todo Initialize message header before publishing
   */
  void deserialize(const uint8_t *data, size_t len, MessageType type)
  {
    // TODO (critical): Initialize message header before publishing
    using namespace ros::serialization;
    IStream input(const_cast<uint8_t*>(data), len);

    switch (type)
    {
      case TOKEN:
      {
        cooperative_driving_networking::Token msg;
        ros::serialization::deserialize(input, msg);
        ROS_DEBUG_STREAM("Deserialized message: " << msg);

        token_pub_.publish(msg);
        break;
      }
      case EMERGENCY_BRAKE:
      {
        cooperative_driving_networking::EmergencyBrake msg;
        ros::serialization::deserialize(input, msg);
        ROS_DEBUG_STREAM("Deserialized message: " << msg);

        emergency_brake_pub_.publish(msg);
        break;
      }
      case CAM:
      {
        cooperative_driving_networking::CooperativeAwarenessMessage msg;
        ros::serialization::deserialize(input, msg);
        ROS_DEBUG_STREAM("Deserialized message: " << msg);

        cam_pub_.publish(msg);
        break;
      }
      case PLATOONING:
      {
        cooperative_driving_networking::Platooning msg;
        ros::serialization::deserialize(input, msg);
        ROS_DEBUG_STREAM("Deserialized message: " << msg);

        platooning_message_pub_.publish(msg);
        break;
      }
      case COMMAND:
      {
        cooperative_driving_msgs::Command msg;
        ros::serialization::deserialize(input, msg);
        ROS_DEBUG_STREAM("Deserialized message: " << msg);

        command_pub_.publish(msg);
        break;
      }
      case UNKNOWN:
      {
        ROS_DEBUG_STREAM("Unknown message type: " << type);
        cooperative_driving_networking::StringStamped msg;
        
        msg.data = reinterpret_cast<char*>(const_cast<uint8_t*>(data));

        string_pub_.publish(msg);
        break;
      }
      default:
        ROS_ERROR_STREAM("Received something not deserializable (unknown type): " << data);
    }
  }

  /** Checks if the node received a frame sent by itself.
   *
   * \param[in]  mac   Frame's mac address
   *
   * \return     True if received own frame
   */
  bool own_frame(const std::string &mac) const
  {
    return mac_address_ == mac;
  }

  /** Check if sender is trusted.
   *
   * The trusted mac addresses are stored in WiFiMonitor::dongles_.
   *
   * \param[in]  mac   Frame's mac address
   *
   * \return     True if sender is trusted, else false
   * 
   * \see     WiFiMonitor::parse_dongle_config
   */
  bool sent_by_trusted_sender(const std::string &mac) const
  {
    return std::find(dongles_.begin(), dongles_.end(), mac) != dongles_.end();
  }

  /** Check if we have already received the same packet.
   *
   * Whether or not a packet has already been received can be determined by checking its
   * sequence number and sender's mac address to the ones of the lastly received packet.
   *
   * \param[in]  seqno       Received message's sequence number.
   * \param[in]  sender_mac  Mac adress of msg's sender.
   *
   * \return     False if either sequence number or mac address differ, true if both are equal to
   * last_received_.seqno_ and last_received_.mac_ or if the sequence number cannot be extracted.
   */
  bool already_received(size_t seqno, const std::string &sender_mac)
  {
    if (seqno == 0)
    {
      // Sequence number could not be extracted: discard packet
      ROS_DEBUG("Could not extract sequence number. Discard packet!");
      return true;
    }

    if ((seqno == last_received_.seqno_) && (sender_mac == last_received_.mac_))
      return true;

    // Either sequence number or sender's mac differ ==> save as new last_received
    last_received_ = { seqno, sender_mac };
    return false;
  }

  std::string mac_address_;           //!< Mac address of attached monitoring dongle.
  std::vector<std::string> dongles_;  //!< List of trusted mac addresses.
  // ros::Timer send_timer_;
  std::string name_;                  //!< Node's name.
  device_t *device_;                  //!< Network device.
  ros::NodeHandle nh_;                //!< ROS node handle.
  tTxOpts tx_options_;                //!< Config of network interface.
  ros::ServiceServer service_;        //!< ROS service, connected to WiFiMonitor::broadcast_message
  std::thread *receiver_thread_;      //!< Thread pointer used to spawn a receiving thread
  struct                              
  {
    size_t seqno_;
    std::string mac_;
  }last_received_;                     //!< Key information of last received packet to avoid duplicates. \see WiFiMonitor::already_received
  Pubscriber<cooperative_driving_networking::StringStamped> string_pub_;  //!< Pubscriber for the string topic.
  Pubscriber<cooperative_driving_networking::Token> token_pub_;           //!< Pubscriber for the token message topic.
  Pubscriber<cooperative_driving_networking::EmergencyBrake> emergency_brake_pub_;  //!< Pubscriber for the emergency brake topic.
  Pubscriber<cooperative_driving_networking::CooperativeAwarenessMessage> cam_pub_;     //!< Pubscriber for the cam topic.
  Pubscriber<cooperative_driving_networking::Platooning> platooning_message_pub_;       //!< Pubscriber for the platooning message topic.
  Pubscriber<cooperative_driving_msgs::Command> command_pub_;       //!< Pubscriber for the platooning message topic.
  const size_t number_of_redundant_packets_ = 5;  //!< Number of duplicate packets sent by method send().
                                                  //!< This is important due do the unreliability of the wireless channel.
  std::unique_ptr<WiFiPacket> message_;           //!< The message that shall be sent. Obviously its content changeds whenever a new message must be sent.
};
}  // namespace cooperative_driving

int main(int argc, char *argv[])
{
  // Extract program argument "name" from roslaunch
  std::string name = ros::getROSArg(argc, argv, "__name");
  if (name == "")
    name = "networking";

  // Set up ROS.
  ros::init(argc, argv, name);

  ROS_INFO_STREAM("Initialize node " << name);

  // Instantiate monitoring node
  cooperative_driving::WiFiMonitor monitor(name);

  // Wait until node is killed or gracefully terminated
  ros::spin();

  return 0;
}

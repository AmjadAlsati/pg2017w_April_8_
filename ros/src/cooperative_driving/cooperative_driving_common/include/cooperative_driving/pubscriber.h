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
#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

#include <ros/node_handle.h>

namespace cooperative_driving
{
/** Wrapper for a advertisement and subscription to the same topic.
 *
 * Provides an interface to access the same topic via both subscription
 * and advertisement. Publishing messages works similary to a ros::Publisher.
 * Messages received on the subscribed will trigger the callback. This
 * however exludes messages published via this Pubscriber.
 */
template <typename T>
class Pubscriber
{
public:
  ros::NodeHandle node_handle_;
  /** Initializes a Pubscriber.
   *
   * \note The interface currently supports only a limited amount of
   * options which can be passed to ros::NodeHandle::advertise() and
   * ros::NodeHandle::subscribe(). In particular, more sophisticated
   * ways of passing the callback are lacking. If more options are
   * required, use ros::SubscribeOptions and ros::AdvertiseOptions.
   * To avoid bloating the class with constructors.
   *
   * \param[in] topic The topic subscribed to/published on.
   * \param[in] input_queue_size Subscriber queue size.
   * \param[in] input_queue_size Publisher queue size.
   * \param[in] callback Function to call with received messages.
   * \param[in] latch (optional) If true, the last message published on this topic will be saved and sent to new
   * subscribers
   * when they connect.
   * \param[in] transport_hints (optional) Transport related options.
   */
  Pubscriber(ros::NodeHandle &node_handle, const std::string &topic, uint32_t input_queue_size,
             uint32_t output_queue_size, const std::function<void(const boost::shared_ptr<T> &)> &callback,
             bool latch = false, const ros::TransportHints &transport_hints = ros::TransportHints())
    : node_handle_(node_handle)
    , topic_(topic)
    , subscriber_(
          node_handle_.subscribe(topic_, input_queue_size, &Pubscriber<T>::internal_callback, this, transport_hints))
    , publisher_(node_handle_.advertise<T>(topic_, output_queue_size, latch))
    , callback_(callback)
    , maximum_messages_in_flight_(input_queue_size + output_queue_size)
  {
  }

  /** Returns the topic accessed by this Pubscriber. */
  std::string get_topic() const
  {
    return topic_;
  }

  /** Publish a message on the topic associated with this Pubscriber.
   *
   * \param[in] message A pointer to the message that should be published
   */
  void publish(const boost::shared_ptr<T> &message)
  {
    publish(*message);
  }

  /** Publish a message on the topic associated with this Pubscriber.
   *
   * \param[in] message The message that should be published
   */
  void publish(const T &message)
  {
    sent_message_timestamps_.push_back(message.header.stamp);
    if (sent_message_timestamps_.size() > maximum_messages_in_flight_)
    {
      sent_message_timestamps_.pop_front();
    }
    publisher_.publish(message);
  }

private:
  /** Calls the configured callback with the received message if it was not
   * sent by this Pubscriber.
   *
   * \param[in] message A pointer to the received message.
   */
  void internal_callback(const boost::shared_ptr<T> &message) const
  {
    if (std::find(sent_message_timestamps_.begin(), sent_message_timestamps_.end(), message->header.stamp) ==
        sent_message_timestamps_.end())
    {
      callback_(message);
    }
  }

  const std::string topic_;                                     //!< The topic used associated with this Pubscriber.
  ros::Subscriber subscriber_;                                  //!< The internal Subscriber.
  ros::Publisher publisher_;                                    //!< The internal Publisher.
  std::function<void(const boost::shared_ptr<T> &)> callback_;  //!< Function called when receiving a message.
  std::deque<ros::Time> sent_message_timestamps_;  //!< Message's timestamps published by this Pubscriber. Used to
                                                   // identify self-sent messages when receiving.
  uint32_t maximum_messages_in_flight_;            //!< Number of messages which might still be unprocessed.
};

}  // namespace cooperative_driving

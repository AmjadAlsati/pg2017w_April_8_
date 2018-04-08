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
#include <gtest/gtest.h>

#include <functional>

#include <geometry_msgs/PointStamped.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include "cooperative_driving/pubscriber.h"

class PubscriberTest : public testing::Test
{
public:
  PubscriberTest()
    : nh()
    , pubsub(nh, "topic", 1, 1, std::bind(&PubscriberTest::pubsub_callback, this, std::placeholders::_1))
    , sub(nh.subscribe<geometry_msgs::PointStamped>(
          "topic", 1, std::bind(&PubscriberTest::sub_callback, this, std::placeholders::_1)))
    , pub(nh.advertise<geometry_msgs::PointStamped>("topic", 1))
    , pubsub_callcount(0)
    , sub_callcount(0)
  {
  }

  ros::NodeHandle nh;
  cooperative_driving::Pubscriber<geometry_msgs::PointStamped> pubsub;
  ros::Subscriber sub;
  ros::Publisher pub;
  uint32_t pubsub_callcount;
  uint32_t sub_callcount;

private:
  void pubsub_callback(const geometry_msgs::PointStampedConstPtr &msg)
  {
    pubsub_callcount += 1;
  }

  void sub_callback(const geometry_msgs::PointStampedConstPtr &msg)
  {
    sub_callcount += 1;
  }
};

TEST_F(PubscriberTest, publish)
{
  pubsub.publish(geometry_msgs::PointStamped());

  ros::Duration(.1).sleep();

  EXPECT_TRUE(sub_callcount == 1);
}

TEST_F(PubscriberTest, publish_shared)
{
  pubsub.publish(boost::make_shared<geometry_msgs::PointStamped>());

  ros::Duration(.1).sleep();

  EXPECT_TRUE(sub_callcount == 1);
}

TEST_F(PubscriberTest, subscribe)
{
  pub.publish(geometry_msgs::PointStamped());

  ros::Duration(.1).sleep();

  EXPECT_TRUE(pubsub_callcount == 1);
}

TEST_F(PubscriberTest, no_self_receive)
{
  pubsub.publish(geometry_msgs::PointStamped());

  ros::Duration(.1).sleep();

  EXPECT_TRUE(pubsub_callcount == 0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_pubscriber_cpp");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}

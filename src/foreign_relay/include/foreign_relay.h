#ifndef FOREIGN_RELAY_H
#define FOREIGN_RELAY_H

// relay just passes messages on. it can be useful if you're trying to ensure
// that a message doesn't get sent twice over a wireless link, by having the
// relay catch the message and then do the fanout on the far side of the
// wireless link.
//
// Copyright (C) 2009, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cstdio>
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"
#include "XmlRpc.h"
#include "ros/xmlrpc_manager.h"
#include "ros/network.h"

using std::string;
using std::vector;
using namespace topic_tools;

static ros::NodeHandle *g_node = NULL;
static bool g_advertised = false;
static string g_foreign_topic;
static string g_local_topic;
static ros::Publisher g_pub;
static string g_host;
static uint32_t g_port = 0;
static bool g_error = false;
typedef enum{MODE_ADV,MODE_SUB} relay_mode_t;
static relay_mode_t g_mode;

#define USAGE "USAGE: foreign_relay {adv|sub} FOREIGN_MASTER_URI FOREIGN_TOPIC LOCAL_TOPIC"
ros::XMLRPCManagerPtr g_xmlrpc_manager = ros::XMLRPCManager::instance();
void foreign_advertise(const std::string &type);
void foreign_unadvertise();
void foreign_subscribe();
void foreign_unsubscribe();
void in_cb(const boost::shared_ptr<ShapeShifter const>& msg);
#endif // FOREIGN_RELAY_H

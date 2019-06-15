/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

/**
 * CPP File containing BSMConvertor method definitions
 */

#include "carma_utils/CARMANode.h"


namespace ros {

  void CARMANode::handleException(const std::exception& e) {
    // Create system alert message
    cav_msgs::SystemAlert alert_msg;
    alert_msg.type = cav_msgs::SystemAlert::FATAL;
    alert_msg.description = "Uncaught Exception in " + ros::this_node::getName() + " exception: " + e.what();
  
    ROS_ERROR_STREAM(alert_msg.description); // Log exception

    //system_alert_pub_.publish(alert_msg); // Notify the rest of the system TODO

    ros::Duration(0.05).sleep(); // Leave a small amount of time for the alert to be published
    shutdown(); // Shutdown this node
  }

void CARMANode::shutdown() {
  std::lock_guard<std::mutex> lock(shutdown_mutex_);
  ROS_WARN_STREAM("Node shutting down");
  shutting_down_ = true;
}

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M), T *obj, const TransportHints &transport_hints=TransportHints()) {
    NodeHandle::subscribe(topic, queue_size, 
      [this](const M& msg) -> void {
          try {
            obj->fp(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      obj, transport_hints);
  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const, T *obj, const TransportHints &transport_hints=TransportHints()) {

  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &), T *obj, const TransportHints &transport_hints=TransportHints()) {

  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &) const, T *obj, const TransportHints &transport_hints=TransportHints()) {

  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M), const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints()) {

  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const, const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints()) {

  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &), const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints()) {

  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &) const, const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints()) {

  }

  template<class M >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(*fp)(M), const TransportHints &transport_hints=TransportHints()) {

  }

  template<class M >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr< M const > &), const TransportHints &transport_hints=TransportHints()) {

  }

  template<class M >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, const boost::function< void(const boost::shared_ptr< M const > &)> &callback, const VoidConstPtr &tracked_object=VoidConstPtr(), const TransportHints &transport_hints=TransportHints()) {

  }

  template<class M , class C >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, const boost::function< void(C)> &callback, const VoidConstPtr &tracked_object=VoidConstPtr(), const TransportHints &transport_hints=TransportHints()) {

  }

  Subscriber 	CARMANode::subscribe(SubscribeOptions &ops) {

  }
}
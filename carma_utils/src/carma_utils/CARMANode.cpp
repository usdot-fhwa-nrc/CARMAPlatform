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

#include <sstream>
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

  bool CARMANode::isRestrictedTopic(const std::string& topic) {
    return system_alert_topic_ == topic;
  }

  void CARMANode::checkSubscriptionInput(const std::string& topic) {
    if (isRestrictedTopic(topic)) {
      ROS_WARN_STREAM("User requested subscription to existing CARMANode reserved topic: " << topic);
    }
  }

  void CARMANode::checkPublisherInput(const std::string& topic) {
    if (isRestrictedTopic(topic)) {
      ROS_WARN_STREAM("User requested publisher to existing CARMANode reserved topic: " << topic);
    }
  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M), T *obj, const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &obj, &fp](const M& msg) mutable -> void {
          try {
            (obj->*fp)(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      this, transport_hints);
  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const, T *obj, const TransportHints &transport_hints) {
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &obj, &fp](const M& msg) -> void {
          try {
            (obj->*fp)(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      this, transport_hints);
  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &), T *obj, const TransportHints &transport_hints) {
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &obj, &fp](const boost::shared_ptr< M const > &msg) mutable -> void {
          try {
            (obj->*fp)(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      this, transport_hints);
  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &) const, T *obj, const TransportHints &transport_hints) {
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &obj, &fp](const boost::shared_ptr< M const > &msg) -> void {
          try {
            (obj->*fp)(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      this, transport_hints);
  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M), const boost::shared_ptr< T > &obj, const TransportHints &transport_hints) {
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &obj, &fp](const M& msg) mutable -> void {
          try {
            (obj->*fp)(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      this, transport_hints);
  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const, const boost::shared_ptr< T > &obj, const TransportHints &transport_hints) {
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &obj, &fp](const M& msg) -> void {
          try {
            (obj->*fp)(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      this, transport_hints);
  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &), const boost::shared_ptr< T > &obj, const TransportHints &transport_hints) {
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &obj, &fp](const boost::shared_ptr< M const > &msg) mutable -> void {
          try {
            (obj->*fp)(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      this, transport_hints);
  }

  template<class M , class T >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &) const, const boost::shared_ptr< T > &obj, const TransportHints &transport_hints) {
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &obj, &fp](const boost::shared_ptr< M const > &msg) -> void {
          try {
            (obj->*fp)(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      this, transport_hints);
  }

  template<class M >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(*fp)(M), const TransportHints &transport_hints) {
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &fp](const M& msg) -> void {
          try {
            fp(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      transport_hints);
  }

  template<class M >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr< M const > &), const TransportHints &transport_hints) {
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &fp](const boost::shared_ptr< M const > & msg) -> void {
          try {
            fp(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      transport_hints);
  }

  template<class M >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, const boost::function< void(const boost::shared_ptr< M const > &)> &callback, const VoidConstPtr &tracked_object, const TransportHints &transport_hints) {
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &callback](const boost::shared_ptr< M const > & msg) -> void {
          try {
            callback(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      tracked_object, transport_hints);
  }

  template<class M , class C >
  Subscriber 	CARMANode::subscribe(const std::string &topic, uint32_t queue_size, const boost::function< void(C)> &callback, const VoidConstPtr &tracked_object, const TransportHints &transport_hints) {
    checkSubscriptionInput(topic);

    return NodeHandle::subscribe(topic, queue_size, 
      [this, &callback](C msg) -> void {
          try {
            callback(msg);
          }
          catch(const std::exception& e) {
            handleException(e);
          }
      }, 
      tracked_object, transport_hints);
  }

  template<class M >
  Publisher CARMANode::advertise (const std::string &topic, uint32_t queue_size, bool latch) {
    checkPublisherInput(topic);
    return NodeHandle::advertise<M>(topic, queue_size, latch);
  }
  
  template<class M >
  Publisher CARMANode::advertise (const std::string &topic, uint32_t queue_size, const SubscriberStatusCallback &connect_cb, const SubscriberStatusCallback &disconnect_cb, const VoidConstPtr &tracked_object, bool latch) {
    checkPublisherInput(topic);
    return NodeHandle::advertise<M>(topic, queue_size, connect_cb, disconnect_cb, tracked_object, latch);
  }
  
  Publisher CARMANode::advertise (AdvertiseOptions &ops) {
    checkPublisherInput(ops.topic);
    return NodeHandle::advertise(ops);
  }

  template<class C>
  void CARMANode::validateCallback(const C& cb) {
    if (!cb) {
      std::ostringstream msg;
      msg << "Invalid callback used in CARMANode: Callback does not point to callable object";
      throw std::invalid_argument(msg.str());
    }
  }

  void CARMANode::validateDelegatedNodeHandle(const CARMANode& cnh) {
    if (&cnh  == this) {
      std::ostringstream msg;
      msg << "Attempted to delegate CARMANode callback to itself ";
      throw std::invalid_argument(msg.str());
    } 
  }

  // TODO add mutex for these setters
  void CARMANode::setSystemAlertCallback(std::function<void(const cav_msgs::SystemAlertConstPtr&)> cb) {
    std::lock_guard<std::mutex> lock(system_alert_mutex_);
    validateCallback<SystemAlertCB>(cb);
    system_alert_cb_ = cb;
  }

  void CARMANode::setSystemAlertCallback(const CARMANode& cnh) {
    validateDelegatedNodeHandle(cnh);
    setSystemAlertCallback(cnh.system_alert_cb_);
  }

  void CARMANode::setShutdownCallback(std::function<void()> cb) {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    validateCallback<ShutdownCB>(cb);
    shutdown_cb_ = cb;
  }

  void CARMANode::setShutdownCallback(const CARMANode& cnh) {
    validateDelegatedNodeHandle(cnh);
    setShutdownCallback(cnh.shutdown_cb_);
  }

  void CARMANode::setExceptionCallback(std::function<void(const std::exception&)> cb) {
    std::lock_guard<std::mutex> lock(exception_mutex_);
    validateCallback<ExceptionCB>(cb);
    exception_cb_ = cb;
  }

  void CARMANode::setExceptionCallback(const CARMANode& cnh) {
    validateDelegatedNodeHandle(cnh);
    setExceptionCallback(cnh.exception_cb_);
  }
}
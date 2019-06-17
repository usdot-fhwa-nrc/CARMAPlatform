#pragma once
/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
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

#include <string>
#include <vector>
#include <exception>
#include <ros/ros.h>
#include <mutex>
#include <memory>
#include <cav_msgs/SystemAlert.h>

namespace ros {
  /**
   * @class ParameterServer
   * @brief An interface which defines the functions needed to access parameters for use in vehicle models.
   * 
   * Supported parameter types match those supported by the ROS 1 C++ parameter interface except for XmlRpc::XmlRpcValue
   */

  class CARMANode: public NodeHandle
  {

    private:
      // Shutdown flags and mutex
      std::mutex shutdown_mutex_;
      std::mutex system_alert_mutex_;
      std::mutex exception_mutex_;
      bool shutting_down_ = false;
      std::string system_alert_topic_;


      std::function<void(const cav_msgs::SystemAlertConstPtr&)> system_alert_cb_;

      std::function<void()> shutdown_cb_;

      std::function<void(const std::exception&)> exception_cb_;

    public:

      CARMANode	(	const std::string & 	ns = std::string(),
        const M_string & 	remappings = M_string() 
      );	

      CARMANode	(	const NodeHandle & 	rhs	);	// Check node handle vs carma node as copy

      CARMANode	(	const NodeHandle & 	parent,
        const std::string & 	ns 
      );

      CARMANode	(	const NodeHandle & 	parent,
        const std::string & 	ns,
        const M_string & 	remappings 
      );

      /**
       * @brief Virtual destructor to ensure delete safety for pointers to implementing classes
       * 
       */
      ~CARMANode() {};

      // TODO overload operator =

      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(M), T *obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const, T *obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &), T *obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &) const, T *obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(M), const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const, const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &), const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &) const, const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(*fp)(M), const TransportHints &transport_hints=TransportHints());
      
      template<class M >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr< M const > &), const TransportHints &transport_hints=TransportHints());
      
      template<class M >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, const boost::function< void(const boost::shared_ptr< M const > &)> &callback, const VoidConstPtr &tracked_object=VoidConstPtr(), const TransportHints &transport_hints=TransportHints());
      
      template<class M , class C >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, const boost::function< void(C)> &callback, const VoidConstPtr &tracked_object=VoidConstPtr(), const TransportHints &transport_hints=TransportHints());
      

      // NOTE: Subscriber subscribe (SubscribeOptions &ops); is not overriden because the target callback cannot be extracted

      template<class M >
      Publisher advertise (const std::string &topic, uint32_t queue_size, bool latch=false);
      
      template<class M >
      Publisher advertise (const std::string &topic, uint32_t queue_size, const SubscriberStatusCallback &connect_cb, const SubscriberStatusCallback &disconnect_cb=SubscriberStatusCallback(), const VoidConstPtr &tracked_object=VoidConstPtr(), bool latch=false);
      
      Publisher advertise (AdvertiseOptions &ops);

      // TODO we need to override service server as well and maybe actions

      using SystemAlertCB = std::function<void(const cav_msgs::SystemAlertConstPtr&)>;
      using ShutdownCB = std::function<void()>;
      using ExceptionCB = std::function<void(const std::exception&)>;


      void setSystemAlertCallback(SystemAlertCB cb);

      void setSystemAlertCallback(const CARMANode& cnh);

      void setShutdownCallback(ShutdownCB cb);

      void setShutdownCallback(const CARMANode& cnh);

      void setExceptionCallback(ExceptionCB cb);

      void setExceptionCallback(const CARMANode& cnh);


      // TODO think about if we should have a run function
      // void onSystemAlert();

      // void onSystemAlert(const CARMANode& cnh);
        // carma_nh_->onSystemAlert(//system alert callback) // Allow each bind to accept a node handle instead. In this case it will delegate the call. This will require check for circular reference
  // carma_nh_->onShutdown(// shutdown callback)
  // carma_nh_->onException(// exception callback)
  // carma_nh_->onRun(// exception callback)

      private:
      /**
       * @brief Handles incoming SystemAlert messages
       * 
       * @param message The message to handle
       * 
       * Handles incoming SystemAlert messages and will shutdown this node if that message was of type SHUTDOWN
       */
      void systemAlertHandler(const cav_msgs::SystemAlertConstPtr& message);

      /**
       * @brief Handles caught exceptions which have reached the top level of this node
       * 
       * @param message The exception to handle
       * 
       * If an exception reaches the top level of this node it should be passed to this function.
       * The function will try to log the exception and publish a FATAL message to system_alert before shutting itself down.
       */
      void handleException(const std::exception& e);

      /**
      * @brief Shutsdown this node
      */
      void shutdown();

      // TODO
      bool isRestrictedTopic(const std::string& topic);

      // TODO
      void checkSubscriptionInput(const std::string& topic);

      void checkPublisherInput(const std::string& topic);

      template<class C>
      void validateCallback(const C& cb);

      void validateDelegatedNodeHandle(const CARMANode& cnh);


  };
}
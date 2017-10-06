/*
 * TODO: Copyright (C) 2017 LEIDOS.
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
 *
 ************************************************************************************
 *
 * This is the worker class that contains all of the logic for the interface manager.
 **/

package gov.dot.fhwa.saxton.carma.interfacemgr;

import org.apache.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

public class InterfaceWorker {

    protected ArrayList<DriverInfo>         drivers_ = new ArrayList<DriverInfo>();
    protected int                           waitTime_ = 10;  //seconds that we must wait after last driver registered
    protected IInterfaceMgr                 mgr_;
    protected Log                           log_;
    protected long                          startedWaiting_;
    protected long							systemReadyTime_;
    protected boolean                       systemOperational_ = false;

    InterfaceWorker(IInterfaceMgr mgr, Log log) {
        mgr_ = mgr;
        log_ = log;
        startedWaiting_ = System.currentTimeMillis();
    }

    /**
     * Stores the desired wait time for drivers to be discovered before declaring the system operational.
     *
     * NOTE: the wait is only approximate. In fact, it is likely that the elapsed time is truncated to the next
     * lowest whole second before this comparison is made, so the actual wait time could be up to 1 sec more
     * than this threshold.
     *
     * @param wait - number of seconds to wait for new drivers to announce themselves
     */
    public void setWaitTime(int wait) {
        waitTime_ = wait;
        if (log_ != null) {
            log_.debug("InterfaceWorker: driver wait time set at " + wait + " seconds.");
        }
    }

    /**
     * Updates the info known about a current driver.  Note that this is expected to be called frequently throughout
     * the life of the node.
     *
     * @param newDriver - all available details about the driver publishing its status
     */
    public void handleNewDriverStatus(DriverInfo newDriver) {
        String name = newDriver.getName();

        //if we already know about this driver then
        int index = getDriverIndex(name);
        if (index >= 0) {

            //if its info has changed then
            if (!newDriver.equalCategoryAndState(drivers_.get(index))) {
                //record the updates. Will need to fetch new driver api as well
                newDriver.setCapabilities(mgr_.getDriverApi(name));
                drivers_.set(index, newDriver);
                log_.debug("InterfaceWorker.handleNewDriverStatus: status changed for " + name);
            }
        //else it's a newly discovered driver
        }else {
            //get its list of capabilities (getDriverApi)
            List<String> cap = mgr_.getDriverApi(name);
            if (cap == null) {
                log_.warn("InterfaceWorker.handleNewDriverStatus: new driver " + name +
                        " has no capabilities! IGNORING.");
            }else {

                //add the info to the list of known drivers
                newDriver.setCapabilities(cap);
                drivers_.add(newDriver);

                //request InterfaceMgr to bind with it
                mgr_.bindWithDriver(name);

                //reset the wait timer
                startedWaiting_ = System.currentTimeMillis();
                log_.info("InterfaceWorker.handleNewDriverStatus: discovered new driver " + name +
                        " with " + cap.size() + " capabilities.");
            }
        }
    }

    /**
     * Updates the status of a driver that has broken its bond.  A broken bond simply indicates a status change for
     * that driver; it may still be alive and functioning, but at a different level of capability (it may have even
     * corrected a previous deficiency, e.g. gone from DEGRADED to fully functional).
     *
     * @param driverName - unique ID of the driver
     */
    public void handleBrokenBond(String driverName) throws IndexOutOfBoundsException {

        //wait a second to make sure that the latest status info from that driver has been published to the
        // discovery topic and therefore updated in our internal "database"
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
        }

        //look up the driver and determine its new set of properties (they will have been stored via the
        // /driver_discovery topic update)
        int index = getDriverIndex(driverName);
        if (index < 0) {
            String msg = "InterfaceWorker.handleBrokenBond can't find driver" + driverName + ". ABORTING.";
            log_.warn(msg);
            throw new IndexOutOfBoundsException(msg);
        }
        DriverInfo driver = drivers_.get(index);

        //if functionality is totally unavailable then
        DriverState state = driver.getState();
        if (state == DriverState.FAULT  ||  state == DriverState.OFF) {
            //remove the driver from the list of available drivers
            drivers_.remove(index);
            log_.warn("InterfaceWorker.handleBrokenBond: driver " + driverName + " is no longer available.");
        }

        //if the system is OPERATIONAL and the new state of this driver is not "fully operational" then
        if (systemOperational_  &&  state != DriverState.OPERATIONAL) {

            //formulate an alert message at the appropriate level depending on the type of driver that is reporting
            // (sendSystemAlert)
            AlertSeverity sev = AlertSeverity.CAUTION;
            String msg = null;

            if (driver.isController()) {
                if (state == DriverState.FAULT  ||  state == DriverState.OFF) {
                    sev = AlertSeverity.FATAL;
                    msg = "Controller driver " + driverName + " is no longer available.";
                }else if (state == DriverState.DEGRADED) {
                    sev = AlertSeverity.WARNING;
                    msg = "Controller driver " + driverName + " is operating at degraded capability.";
                }
            }

            String level = (state == DriverState.DEGRADED) ? "degraded" : "gone";
            if (driver.isPosition()) {
                sev = AlertSeverity.WARNING;
                msg = "Position driver " + driverName + " is " + level;
            }

            if (driver.isComms()) {
                sev = AlertSeverity.WARNING;
                msg = "Comms driver " + driverName + " is " + level;
            }

            if (driver.isSensor()  ||  driver.isCan()) {
                sev = AlertSeverity.CAUTION;
                msg = "Driver " + driverName + " is " + level;
            }

            mgr_.sendSystemAlert(sev, msg);
        }
    }

    /**
     * Returns a list of capabilities provided by drivers that each provide all of the requested capabilities
     * once the system is OPERATIONAL.
     * Note that each capability in the input list may be of the form
     *     [name]
     * or of the form
     *     [driver category]/[name]
     * If the driver category is not specified then we will assume any category of driver is acceptable. If it
     * is specified, then only drivers of that category will be considered.
     *
     * @param requestedCapabilities - a list of capabilities that must be met (inclusive)
     * @return - a list of fully-qualified driver names and capabilities where each driver involved can satisfy
     * all of the requestedCapabilities
     */
    public List<String> getDrivers(List<String> requestedCapabilities) {
        List<String> result = new ArrayList<String>();

        //if the system is ready for operation then
        if (systemOperational_) {

            //loop through all known drivers
            for (int driverIndex = 0;  driverIndex < drivers_.size();  ++driverIndex) {
                DriverInfo driver = drivers_.get(driverIndex);
                List<String> driverCaps = driver.getCapabilities();
                List<String> tentativeResult = new ArrayList<>();

                //loop through all requested capabilities
                boolean foundAllCapabilities = true;
                for (int req = 0;  req < requestedCapabilities.size();  ++req) {

                    //get the requested driver category, if any, and the requested capability
                    String[] items = requestedCapabilities.get(req).split("/");
                    DriverCategory cat = DriverCategory.UNDEFINED;
                    if (items.length > 1) {
                        cat = DriverCategory.getCat(items[items.length - 2]);
                    }
                    String reqCapability = items[items.length - 1];

                    //if this driver cannot provide this capability then break out of loop
                    boolean foundThisCapability = false;
                    if (driver.hasCategory(cat)) {
                        for (int capIndex = 0;  capIndex < driverCaps.size();  ++capIndex) {
                            String[] capBreakout = driverCaps.get(capIndex).split("/");
                            String driverCap = capBreakout[capBreakout.length - 1];
                            if (reqCapability.equals(driverCap)) {
                                foundThisCapability = true;
                                tentativeResult.add(driverCaps.get(capIndex));
                                break;
                            }
                        }
                    }
                    if (!foundThisCapability) {
                        foundAllCapabilities = false;
                        break;
                    }
                }

                //if the driver is satisfactory then
                if (foundAllCapabilities) {
                    //add them all to the return list
                    result.addAll(tentativeResult);
                }
            }
        }

        return result;
    }

    /**
     * Indicates whether the system has just become OPERATIONAL.  If it is either not ready or it has
     * been OPERATIONAL in a previous call then it will return false.
     * Waits an additional amount of time after the latest detected driver in case any further drivers
     * come on line.
     *
     * @return - true if the system is newly OPERATIONAL (one call only will return true)
     */
    public boolean isSystemReady() {

        //if system is not yet OPERATIONAL then
        if (!systemOperational_) {

            //if wait timer has expired then
            long elapsed = System.currentTimeMillis() - startedWaiting_;
            if (elapsed > 1000*waitTime_) {

                //indicate that it is now OPERATIONAL
                systemOperational_ = true;
                //log the time required to get to this point
                log_.info("///// InterfaceWorker declaring SYSTEM OPERATIONAL.");
                log_.info("---elapsed = " + elapsed + ", waitTime = " + waitTime_);
                
                //record the time of this event
                systemReadyTime_ = System.currentTimeMillis();
            }
        }

        return systemOperational_;
    }
    
    
    /**
     * Returns the time since this class declared the system ready for operation.
     * @return time in ms
     */
    public long timeSinceSystemReady() {
    	return System.currentTimeMillis() - systemReadyTime_;
    }

    //////////

    /**
     * Returns the index in the drivers_ array that matches the name of the given driver.
     *
     * @param givenName - the one we are looking for
     * @return - index of the driver that matches given
     */
    protected int getDriverIndex(String givenName) {

        if (drivers_.size() > 0) {
            for (int i = 0;  i < drivers_.size();  ++i) {
                if (drivers_.get(i).getName().equals(givenName)) {
                    return i;
                }
            }
        }

        return -1;
    }
}

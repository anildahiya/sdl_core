/*
 * Application.h
 *
 *  Created on: Oct 4, 2012
 *      Author: vsalo
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

#include "IApplication.h"
#include "JSONHandler/ALRPCObjects/Language.h"
#include "JSONHandler/ALRPCObjects/SyncMsgVersion.h"
#include "JSONHandler/ALRPCObjects/HMILevel.h"
#include "JSONHandler/ALRPCObjects/AudioStreamingState.h"
#include <string>
#include <vector>

namespace log4cplus
{
    class Logger;
}

namespace NsAppManager
{

/**
 * \brief class Application acts as a metaphor for every mobile application being registered on HMI
 */
class Application : public IApplication
{
public:

    /**
     * \brief Default class constructor
     */
    Application( const std::string& name, unsigned char sessionId );

    /**
     * \brief Default class destructor
     */
	virtual ~Application( );

    /**
     * \brief Set application priority
     * \param priority a priority to set
     */
	virtual void setApplicationPriority( const AppPriority& priority );

    /**
     * \brief Set application HMI status level
     * \param hmiLevel HMI status level of application
     */
	void setApplicationHMIStatusLevel( const AppLinkRPC::HMILevel::HMILevelInternal& hmiLevel );

    /**
     * \brief retrieve aplication HMI status level
     * \return HMI status level of application
     */
	const AppLinkRPC::HMILevel::HMILevelInternal& getApplicationHMIStatusLevel( ) const;

    /**
     * \brief Set application audio streaming state
     * \param streamingState audio streaming state of application
     */
	void setApplicationAudioStreamingState( const AppLinkRPC::AudioStreamingState& hmiLevel );

    /**
     * \brief retrieve application audio streaming state
     * \return application audio streaming state
     */
	const AppLinkRPC::AudioStreamingState& getApplicationAudioStreamingState( ) const;

    /**
     * \brief Set application NGN media screen app name
     * \param value application NGN media screen app name
     */
	void setNgnMediaScreenAppName(const std::string& value);

    /**
     * \brief Set application voice recognition synonyms
     * \param value application voice recognition synonyms
     */
	void setVrSynonyms(const std::vector<std::string>& value);

    /**
     * \brief Set application usage of vehicle data
     * \param value does the application use vehicle data
     */
	void setUsesVehicleData(bool value);

    /**
     * \brief Set if the application is a media application
     * \param value is the application a media application
     */
	void setIsMediaApplication(bool value);

    /**
     * \brief Set application desired languuage
     * \param value application desired language
     */
	void setLanguageDesired(AppLinkRPC::Language value);

    /**
     * \brief Set application autoactivate ID
     * \param value application autoactivate ID
     */
	void setAutoActivateID(const std::string& value);

    /**
     * \brief Set application sync message version
     * \param value application sync message version
     */
	void setSyncMsgVersion(AppLinkRPC::SyncMsgVersion value);

    /**
     * \brief Set application ID
     * \param value application ID
     */
    void setAppID( const std::string& value );

    /**
     * \brief Set application HMI desired display language
     * \param value application HMI desired display language
     */
    void setHMIDisplayLanguageDesired( AppLinkRPC::Language value );

    /**
     * \brief retrieve application NGN media screen application name
     * \return application NGN media screen application name
     */
	const std::string& getNgnMediaScreenAppName( ) const;

    /**
     * \brief retrieve application voice-recognition synonyms
     * \return application voice-recognition synonyms
     */
	const std::vector<std::string>& getVrSynonyms( ) const;

    /**
     * \brief retrieve does the application use vehicle data
     * \return does the application use vehicle data
     */
	bool getUsesVehicleData( ) const;

    /**
     * \brief retrieve is the application a media application
     * \return is the application a media application
     */
	bool getIsMediaApplication( ) const;

    /**
     * \brief retrieve application desired language
     * \return application desired language
     */
	const AppLinkRPC::Language& getLanguageDesired( ) const;

    /**
     * \brief retrieve application auto-activate ID
     * \return application auto-activate ID
     */
	const std::string& getAutoActivateID( ) const;

    /**
     * \brief retrieve application sync message version
     * \return application sync msg version
     */
	const AppLinkRPC::SyncMsgVersion& getSyncMsgVersion( ) const;

    /**
     * \brief retrieve application ID
     * \return application ID
     */
    const std::string& getAppID( ) const;

    /**
     * \brief retrieve application HMI desired display language
     * \return application HMI desired display language
     */
    const AppLinkRPC::Language& getHMIDisplayLanguageDesired( ) const;

    /**
     * \brief retrieve application session ID
     * \return application session ID
     */
    unsigned char getSessionID() const;

private:

    /**
     * \brief Copy constructor
     */
	Application(const Application& );
	
    const unsigned char mSessionID;
	std::string mNgnMediaScreenAppName;
	std::vector<std::string> mVrSynonyms;
	bool m_bUsesVehicleData;
	bool m_bIsMediaApplication;
	AppLinkRPC::Language mLanguageDesired;
	std::string mAutoActivateID;
	AppLinkRPC::SyncMsgVersion mSyncMsgVersion;
	AppLinkRPC::HMILevel::HMILevelInternal mHMIStatusLevel;
	AppLinkRPC::AudioStreamingState mAudioStreamingState;
    std::string mAppID;
    AppLinkRPC::Language mHMIDisplayLanguageDesired;

    static log4cplus::Logger mLogger;
};

} // namespace NsAppManager

#endif /* APPLICATION_H_ */

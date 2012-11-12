/*
 * Application.cpp
 *
 *  Created on: Oct 4, 2012
 *      Author: vsalo
 */

#include "AppMgr/Application.h"
#include "LoggerHelper.hpp"

namespace NsAppManager
{
	
log4cplus::Logger Application::mLogger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("Application"));

/**
 * \brief Default class constructor
 */
Application::Application(const std::string& name , unsigned char sessionId)
	:IApplication(name)
	,mNgnMediaScreenAppName("")
	,m_bUsesVehicleData(false)
	,m_bIsMediaApplication(false)
	,mAutoActivateID("")
    ,mSessionID(sessionId)
{
    LOG4CPLUS_INFO_EXT(mLogger, " Created an application "<< name <<" for the session id "<<sessionId);
}

/**
 * \brief Copy constructor
 */
Application::Application( const Application& app )
	:IApplication(app.getName())
	,mNgnMediaScreenAppName(app.getNgnMediaScreenAppName())
	,m_bUsesVehicleData(app.getUsesVehicleData())
	,m_bIsMediaApplication(app.getIsMediaApplication())
	,mAutoActivateID(app.getAutoActivateID())
	,mLanguageDesired(app.getLanguageDesired())
	,mHMIStatusLevel(app.getApplicationHMIStatusLevel())
	,mSyncMsgVersion(app.getSyncMsgVersion())
    ,mSessionID(app.getSessionID())
{
	mVrSynonyms = app.getVrSynonyms();
}

/**
 * \brief Default class destructor
 */
Application::~Application( )
{
    LOG4CPLUS_INFO_EXT(mLogger, " Deleted an application "<< mName << " session id "<<mSessionID);
}

/**
 * \brief Set application priority
 * \param priority a priority to set
 */
void Application::setApplicationPriority( const AppPriority& priority )
{
	mPriority = priority;
}

/**
 * \brief retrieve aplication HMI status level
 * \return HMI status level of application
 */
const AppLinkRPC::HMILevel::HMILevelInternal& Application::getApplicationHMIStatusLevel( ) const
{
	return mHMIStatusLevel;
}

/**
 * \brief Set application HMI status level
 * \param hmiLevel HMI status level of application
 */
void Application::setApplicationHMIStatusLevel( const AppLinkRPC::HMILevel::HMILevelInternal& hmiLevel )
{
	mHMIStatusLevel = hmiLevel;
}

/**
 * \brief retrieve application audio streaming state
 * \return application audio streaming state
 */
const AppLinkRPC::AudioStreamingState& Application::getApplicationAudioStreamingState( ) const
{
	return mAudioStreamingState;
}

/**
 * \brief Set application audio streaming state
 * \param streamingState audio streaming state of application
 */
void Application::setApplicationAudioStreamingState( const AppLinkRPC::AudioStreamingState& streamingState )
{
	mAudioStreamingState = streamingState;
}

/**
 * \brief Set application NGN media screen app name
 * \param value application NGN media screen app name
 */
void Application::setNgnMediaScreenAppName(const std::string& value)
{
	mNgnMediaScreenAppName = value;
}

/**
 * \brief Set application voice recognition synonyms
 * \param value application voice recognition synonyms
 */
void Application::setVrSynonyms(const std::vector<std::string>& value)
{
	mVrSynonyms = value;
}

/**
 * \brief Set application usage of vehicle data
 * \param value does the application use vehicle data
 */
void Application::setUsesVehicleData(bool value)
{
	m_bUsesVehicleData = value;
}

/**
 * \brief Set if the application is a media application
 * \param value is the application a media application
 */
void Application::setIsMediaApplication(bool value)
{
	m_bIsMediaApplication = value;
}

/**
 * \brief Set application desired languuage
 * \param value application desired language
 */
void Application::setLanguageDesired(AppLinkRPC::Language value)
{
	mLanguageDesired = value;
}

/**
 * \brief Set application autoactivate ID
 * \param value application autoactivate ID
 */
void Application::setAutoActivateID(const std::string& value)
{
	mAutoActivateID = value;
}

/**
 * \brief Set application sync message version
 * \param value application sync message version
 */
void Application::setSyncMsgVersion(AppLinkRPC::SyncMsgVersion value)
{
    mSyncMsgVersion = value;
}

/**
 * \brief Set application ID
 * \param value application ID
 */
void Application::setAppID(const std::string &value)
{
    mAppID = value;
}

/**
 * \brief Set application HMI desired display language
 * \param value application HMI desired display language
 */
void Application::setHMIDisplayLanguageDesired(AppLinkRPC::Language value)
{
    mHMIDisplayLanguageDesired = value;
}

/**
 * \brief retrieve application NGN media screen application name
 * \return application NGN media screen application name
 */
const std::string& Application::getNgnMediaScreenAppName( ) const
{
	return mNgnMediaScreenAppName;
}

/**
 * \brief retrieve application voice-recognition synonyms
 * \return application voice-recognition synonyms
 */
const std::vector<std::string>& Application::getVrSynonyms( ) const
{
	return mVrSynonyms;
}

/**
 * \brief retrieve does the application use vehicle data
 * \return does the application use vehicle data
 */
bool Application::getUsesVehicleData( ) const
{
	return m_bUsesVehicleData;
}

/**
 * \brief retrieve is the application a media application
 * \return is the application a media application
 */
bool Application::getIsMediaApplication( ) const
{
	return m_bIsMediaApplication;
}

/**
 * \brief retrieve application desired language
 * \return application desired language
 */
const AppLinkRPC::Language& Application::getLanguageDesired( ) const
{
	return mLanguageDesired;
}

/**
 * \brief retrieve application auto-activate ID
 * \return application auto-activate ID
 */
const std::string& Application::getAutoActivateID( ) const
{
	return mAutoActivateID;
}

/**
 * \brief retrieve application sync message version
 * \return application sync msg version
 */
const AppLinkRPC::SyncMsgVersion& Application::getSyncMsgVersion( ) const
{
    return mSyncMsgVersion;
}

/**
 * \brief retrieve application ID
 * \return application ID
 */
const std::string &Application::getAppID() const
{
    return mAppID;
}

/**
 * \brief retrieve application HMI desired display language
 * \return application HMI desired display language
 */
const AppLinkRPC::Language &Application::getHMIDisplayLanguageDesired() const
{
    return mHMIDisplayLanguageDesired;
}

/**
 * \brief retrieve application session ID
 * \return application session ID
 */
unsigned char Application::getSessionID() const
{
    return mSessionID;
}

}

#include "MainSettings.hpp"

namespace Application
{
	/**
	 *
	 */
	MainSettings::MainSettings() : drawOpenSet(true), speed(10), worldNumber(0), particleFilterOn(false), kalmanFilterOn(false)
	{
	}
	/**
	 *
	 */
	MainSettings::~MainSettings()
	{
	}
	/**
	 *
	 */
	bool MainSettings::getDrawOpenSet() const
	{
		return drawOpenSet;
	}
	/**
	 *
	 */
	void MainSettings::setDrawOpenSet( bool aDrawOpenSet)
	{
		drawOpenSet = aDrawOpenSet;
	}
	/**
	 *
	 */
	unsigned long MainSettings::getSpeed() const
	{
		return speed;
	}
	/**
	 *
	 */
	void MainSettings::setSpeed( unsigned long aSpeed)
	{
		speed = aSpeed;
	}
	/**
	 *
	 */
	unsigned long MainSettings::getWorldNumber() const
	{
		return worldNumber;
	}
	/**
	 *
	 */
	void MainSettings::setWorldNumber( unsigned long aWorldNumber)
	{
		worldNumber = aWorldNumber;
	}

	void MainSettings::setParticleFilterOn(bool on) {
		this->particleFilterOn = on;
	}

	bool MainSettings::getParticleFilterOn() const {
		return this->particleFilterOn;
	}

	void MainSettings::setKalmanFilterOn(bool on) {
		this->kalmanFilterOn = on;
	}

	bool MainSettings::getKalmanFilterOn() const {
		return this->kalmanFilterOn;
	}
} /* namespace Application */

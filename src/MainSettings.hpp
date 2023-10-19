#ifndef MAINSETTINGS_HPP_
#define MAINSETTINGS_HPP_

#include "Config.hpp"

namespace Application
{

	/*
	 *
	 */
	class MainSettings
	{
		public:
			/**
			 *
			 */
			MainSettings();
			/**
			 *
			 */
			virtual ~MainSettings();
			/**
			 *
			 */
			bool getDrawOpenSet() const;
			/**
			 *
			 */
			void setDrawOpenSet( bool aDrawOpenSet);
			/**
			 *
			 */
			unsigned long getSpeed() const;
			/**
			 *
			 */
			void setSpeed( unsigned long aSpeed);
			/**
			 *
			 */
			unsigned long getWorldNumber() const;
			/**
			 *
			 */
			void setWorldNumber( unsigned long aWorldNumber);

			void setParticleFilterOn(bool on);

			bool getParticleFilterOn() const;

			bool getKalmanFilterOn() const;

			void setKalmanFilterOn(bool on);

		private:
			bool drawOpenSet;
			unsigned long speed;
			unsigned long worldNumber;
			bool particleFilterOn;
			bool kalmanFilterOn;
	};

} /* namespace Application */

#endif /* SRC_MAINSETTINGS_HPP_ */

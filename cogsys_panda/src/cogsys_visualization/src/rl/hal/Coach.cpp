//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <cassert>
#include <string>
#include <rl/util/Timer.h>

#include "Coach.h"

namespace rl
{
	namespace hal
	{
		Coach::Coach(
			const ::std::size_t& dof,
			const ::rl::math::Real& updateRate,
			const ::std::size_t& i,
			const ::std::string& address,
			const unsigned short int& port
		) :
			AxisController(dof, updateRate),
			JointPositionActuator(dof, updateRate),
			JointTorqueActuator(dof, updateRate),
			JointVelocityActuator(dof, updateRate),
			i(i),
			socket(Socket::Tcp4(address, port)),
			text()
		{
		}
		
		Coach::~Coach()
		{
		}
		
		void
		Coach::close()
		{
			this->socket.close();
			this->setConnected(false);
		}
		
		void
		Coach::open()
		{
			this->socket.open();
			this->socket.connect();
			this->setConnected(true);
		}
		
		void
		Coach::setJointPosition(const ::rl::math::Vector& q)
		{
			assert(this->getDof() >= q.size());
			
			this->text.clear();
			this->text.str("");
			this->text << 2 << " " << this->i;
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				this->text << " " << q(i);
			}
			
			this->text << ::std::endl;
		}
		
		void
		Coach::setJointTorque(const ::rl::math::Vector& tau)
		{
			assert(this->getDof() >= tau.size());
			
			this->text.clear();
			this->text.str("");
			this->text << 5 << " " << this->i;
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				this->text << " " << tau(i);
			}
			
			this->text << ::std::endl;
		}
		
		void
		Coach::setJointVelocity(const ::rl::math::Vector& qd)
		{
			assert(this->getDof() >= qd.size());
			
			this->text.clear();
			this->text.str("");
			this->text << 3 << " " << this->i;
			
			for (::std::size_t i = 0; i < this->getDof(); ++i)
			{
				this->text << " " << qd(i);
			}
			
			this->text << ::std::endl;
		}
		
		void
		Coach::start()
		{
			this->setRunning(true);
		}
		
		void
		Coach::step()
		{
			this->socket.write(text.str().c_str(), text.str().length());
			::rl::util::Timer::sleep(this->getUpdateRate());
		}
		
		void
		Coach::stop()
		{
			this->setRunning(false);
		}
	}
}

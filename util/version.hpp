/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file versions.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Code version
 *
 ******************************************************************************/


#ifndef VERSIONS_HPP_
#define VERSIONS_HPP_

namespace version
{

    // Version of MAVRIC Library
    const uint32_t mavric =   (1 << 24)   // major
                            + (5 << 16)   // minor
                            + (0 << 8)    // patch
                            + (0 << 0);   // release

    // Git hash for MAVRIC Library
    #ifdef MAVRIC_GIT_HASH
        const uint8_t mavric_git_hash[8] = MAVRIC_GIT_HASH;
    #else
        const uint8_t mavric_git_hash[8] = "unknown";
    #endif

    // Version of project
    #ifdef PROJECT_VERSION
        const uint32_t project = PROJECT_VERSION;
    #else
        const uint32_t project = 0;
    #endif

    // Git hash of project
    #ifdef PROJECT_GIT_HASH
        const uint8_t project_git_hash[8] = PROJECT_GIT_HASH;
    #else
        const uint8_t project_git_hash[8] = "unknown";
    #endif

    // Project name
    #ifdef PROJECT_NAME
        const uint8_t project_name[] = PROJECT_NAME;
    #else
        const uint8_t project_name[] = "unknown";
    #endif
}

#endif /* VERSIONS_HPP_ */

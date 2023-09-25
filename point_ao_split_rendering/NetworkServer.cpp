/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include "NetworkServer.h"

namespace split_rendering {

NetworkServer::~NetworkServer() {
  //stopThreads();
}

int NetworkServer::establishConnection() 
{
  //if (!server_.acceptConnection(port_)) {
  //  return -1;
  //}

  return 0;
}

} // namespace split_rendering

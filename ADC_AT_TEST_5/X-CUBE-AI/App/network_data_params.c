/**
  ******************************************************************************
  * @file    network_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Mon Nov 20 16:55:24 2023
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#include "network_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_network_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_network_weights_array_u64[106] = {
  0xbf18f00a3f220b5cU, 0x402e405cc089a4f6U, 0x402c9597c13e4d44U, 0x407cb5bfc1502624U,
  0x3f670856c10e20ccU, 0x4119bed940de54e8U, 0x4181f27cbed7b816U, 0x41a4208ac0a0fbeeU,
  0x41247f0440aaab9eU, 0x3e3499ecc09012c0U, 0x40716065c122ba2eU, 0xbe10ab1c406b89caU,
  0x40863f6040f94c8bU, 0x411c07b9c10e75b5U, 0x4089f82fc10a3c7aU, 0xbfad913f3fa5cd5fU,
  0xc15ef361bfe48a61U, 0x415a7a06404a8834U, 0x403f5a5abf74b4daU, 0x3f149f60be6bef5dU,
  0x3ea58bc0be5e3939U, 0x3ab2fff6bf125cbfU, 0xbd86217e3edc7066U, 0xbf3e8e6d3edd9ed4U,
  0x3f3780383e36576bU, 0x3f0a1403bec7c0ffU, 0xbed32c39bf312f13U, 0xbeccd7743f1d86f6U,
  0xbeb7f1953ee9ce0cU, 0x3f36ba5a3dc1aadaU, 0x3f25526dbf06fe01U, 0xbe9f88d3bebaf3f5U,
  0xbed6ec9b3ef202eeU, 0xbf4b81813f0ab365U, 0x3f43d7523e501801U, 0x3f1892d2bd8b14bfU,
  0xbeacd839be6f8c50U, 0xbf10f1953f131b04U, 0xbecf5be23f040fa9U, 0x3f30bcaa3e30ebe4U,
  0x3e4c3025be54b08dU, 0xbf1bc14cbf2b0923U, 0xbcbc97133f397216U, 0xbe9079ac3f27dca8U,
  0xbdb5a1edbf502a81U, 0xbebc0ad53dc1dcffU, 0x3ebb160e3deea5ceU, 0x3f1cfc50be256509U,
  0x3f0e1b15bee9ec8aU, 0xbd96b46dbf3b05f3U, 0xbf1db9243df32aaaU, 0x3eba95a03ef3b6aeU,
  0x3efa9db63e7ea752U, 0x3f043563bf67a5cdU, 0xbed7faa7bf0eec6eU, 0xbeeda31a3e05b12fU,
  0x3ef3e878be287836U, 0x3ef7371c3ee3b194U, 0x3e22a53abe9b4c2fU, 0x3b68351abcd02dd4U,
  0xbd9ebaea3d4ae3b4U, 0x3e1cc87a3f38453eU, 0x3e33715dbeaa79aeU, 0x3db12320be94b96cU,
  0xbeb928fdbf213193U, 0xbe5d43e2beab31aeU, 0x3d9ede0f3dcf5dd8U, 0x3ef778223f541782U,
  0xbd16d0c4bc8b0a79U, 0xbe3c7e813dc8f44bU, 0xbe0c7a683e7b6bdeU, 0xbd40432f3de2eadbU,
  0x3e651328bf550785U, 0x3e5263da3ebbc492U, 0xbf12ac4d3f257c2dU, 0x3d3166d83e082d36U,
  0xbe33566ebc18a6e1U, 0xbc30d191bebd7c8cU, 0x3ea980043ce7e874U, 0xbf3347cd3efba0c1U,
  0xbe5b9c773da40a36U, 0x3dc457dd3e55262aU, 0x3e68356dbe85d93eU, 0xbd9fdd77bd313611U,
  0xbeed7f3e3f141630U, 0xbee7285c3ee17aa5U, 0xbd3edf9e3e876a9eU, 0x3ea77e8cbe7c1abdU,
  0x3ee0b3863dbc6739U, 0xbe9c23683eb14a3cU, 0xbe83aadf3f0e215cU, 0x3da93f113f251192U,
  0xbebfaa23bf33882bU, 0x3e7a5f383df5fa92U, 0x3e0d6a9fbed0e786U, 0x3e58f01c3e963771U,
  0xbf5f57eb3ca352b4U, 0xbf74d279bf6cccfbU, 0x3cdc6530bf142516U, 0x3e9e656abe70024bU,
  0x3d90207a3e6c938dU, 0x3d2f7e8cbf000ac5U, 0xbcebb36e3d56e9f0U, 0xbe039733bf7e1764U,
  0x3ea77486bd874dbcU, 0x3e0abb6bU,
};


ai_handle g_network_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_network_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};


/*
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: Copyright (c) 2013-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDE_UART_PROTO_H
#define INCLUDE_UART_PROTO_H

/**
 * @defgroup virt_uart_protocol_codes Virtualized UART Protocol Codes
 * @{
 */
/// Specifies the character to use as a beginning of an ESC sequence.
/// The character afterwards is a "command"
#define UART_PROTO_ESC_START 0xffU

/// Command send actual character that is being used for ESC start
/// (Think %% in printfs)
#define UART_PROTO_ESC_ESC   0xffU

/// CH < 16 are used to switch destination to guest number CH

/// Switch to "hypervisor output"
#define UART_PROTO_ESC_ID_HYP 0xfeU

/// Specifies to restart.
/// Party sending this command just restarted and will assume start
/// conditions
#define UART_PROTO_ESC_RESET   0xfdU

/// Specifies to query the list of guest names.
#define UART_PROTO_ESC_NAME_QUERY 0xfcU
/// Specifies to reply to the list of guest names.
#define UART_PROTO_ESC_NAME_REPLY 0xfbU
/// Specifies to delimit the list of guest names.
#define UART_PROTO_ESC_NAME_DELIM 0xfaU

/** @} */

#endif

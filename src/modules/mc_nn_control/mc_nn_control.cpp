/****************************************************************************
 *
 *   Copyright (c) 2013-2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file mc_nn_control.cpp
 * Multicopter Neural Network Control module, from position setpoints to control allocator.
 *
 * @author Sindre Meyer Hegre <sindre.hegre@gmail.com>
 */

#include "mc_nn_control.hpp"

namespace {
using NNControlOpResolver = tflite::MicroMutableOpResolver<1>;

TfLiteStatus RegisterOps(NNControlOpResolver& op_resolver) {
  TF_LITE_ENSURE_STATUS(op_resolver.AddFullyConnected());
  return kTfLiteOk;
}
}  // namespace

extern "C" __EXPORT int mc_nn_control_main(int argc, char *argv[])
{
	PX4_INFO("Hello from mc_nn_control_main");

	// Load the model
	const tflite::Model* model = ::tflite::GetModel(simple_net_tflite);  // Replace with your model data variable
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		PX4_ERR("Model provided is schema version %d not equal to supported version %d.",
			model->version(), TFLITE_SCHEMA_VERSION);
		return -1;
	}

	// Set up the interpreter
	static NNControlOpResolver resolver;
	if (RegisterOps(resolver) != kTfLiteOk) {
		PX4_ERR("Failed to register ops");
		return -1;
	}
	constexpr int kTensorArenaSize = 10 * 1024;
	static uint8_t tensor_arena[kTensorArenaSize];
	tflite::MicroInterpreter interpreter(model, resolver, tensor_arena, kTensorArenaSize);

	// Allocate memory for the model's tensors
	TfLiteStatus allocate_status = interpreter.AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		PX4_ERR("AllocateTensors() failed");
		return -1;
	}

	// Prepare input data
	TfLiteTensor* input = interpreter.input(0);
	if (input == nullptr) {
		PX4_ERR("Input tensor is null");
		return -1;
	}

	// Fill input tensor with 10 numbers between 0 and 1
	for (int i = 0; i < 10; ++i) {
		input->data.f[i] = static_cast<float>(i) / 10.0f;
	}

	// Run inference
	TfLiteStatus invoke_status = interpreter.Invoke();
	if (invoke_status != kTfLiteOk) {
		PX4_ERR("Invoke() failed");
		return -1;
	}

	// Print the output
	TfLiteTensor* output = interpreter.output(0);
	if (output == nullptr) {
		PX4_ERR("Output tensor is null");
		return -1;
	}

	PX4_INFO("Output:");
	for (int i = 0; i < output->dims->data[0]; ++i) {
		PX4_INFO("output[%d] = %f", i, static_cast<double>(output->data.f[i]));
	}

	return 0;
}

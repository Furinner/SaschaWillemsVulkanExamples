/*
* Basic camera class providing a look-at and first-person camera
*
* Copyright (C) 2016-2024 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera
{
private:

	void updateViewMatrix()
	{
		glm::mat4 currentMatrix = matrices.view;

		/*glm::mat4 rotM = glm::mat4(1.0f);
		glm::mat4 transM;

		rotM = glm::rotate(rotM, glm::radians(rotation.x * (flipY ? -1.0f : 1.0f)), glm::vec3(1.0f, 0.0f, 0.0f));
		rotM = glm::rotate(rotM, glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
		rotM = glm::rotate(rotM, glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));

		glm::vec3 translation = position;
		if (flipY) {
			translation.y *= -1.0f;
		}
		transM = glm::translate(glm::mat4(1.0f), translation);

		if (type == CameraType::firstperson)
		{
			matrices.view = rotM * transM;
		}
		else
		{
			matrices.view = transM * rotM;
		}

		viewPos = glm::vec4(position, 0.0f) * glm::vec4(-1.0f, 1.0f, -1.0f, 1.0f);*/

		matrices.view = glm::lookAt(position, position + getCamFront(), glm::vec3(0, 1, 0));
		if (matrices.view != currentMatrix) {
			updated = true;
		}
	};
public:
	float fov;
	float znear, zfar;
	float orthoLeft, orthoRight, orthoBottom, orthoTop;
	float aspect;
	bool orthographic = false;
	//for debug
	//left dx+, right dx-, up dy+£¬down dy-
	//float dx, dy;
	enum CameraType { lookat, firstperson };
	CameraType type = CameraType::lookat;

	glm::vec3 rotation = glm::vec3();
	glm::vec3 position = glm::vec3();
	glm::vec4 viewPos = glm::vec4();

	float rotationSpeed = 1.0f;
	float movementSpeed = 1.0f;

	bool updated = true;
	bool flipY = false;

	struct
	{
		glm::mat4 perspective;
		glm::mat4 view;
	} matrices;

	struct
	{
		bool left = false;
		bool right = false;
		bool up = false;
		bool down = false;
	} keys;

	bool moving() const
	{
		return keys.left || keys.right || keys.up || keys.down;
	}

	float getNearClip() const {
		return znear;
	}

	float getFarClip() const {
		return zfar;
	}

	void setPerspective(float fov, float aspect, float znear, float zfar)
	{
		glm::mat4 currentMatrix = matrices.perspective;
		this->fov = fov;
		this->znear = znear;
		this->zfar = zfar;
		matrices.perspective = glm::perspective(glm::radians(fov), aspect, znear, zfar);
		if (flipY) {
			//matrices.perspective[1][1] *= -1.0f;
		}
		if (matrices.view != currentMatrix) {
			updated = true;
		}
	};

	void updatePerspective(float fov, float aspect, float znear, float zfar) {
		this->fov = fov;
		this->fov = fov;
		this->znear = znear;
		this->zfar = zfar;
		matrices.perspective = glm::perspective(glm::radians(fov), aspect, znear, zfar);
	}

	void updateAspectRatio(float aspect)
	{
		glm::mat4 currentMatrix = matrices.perspective;
		matrices.perspective = glm::perspective(glm::radians(fov), aspect, znear, zfar);
		if (flipY) {
			//matrices.perspective[1][1] *= -1.0f;
		}
		if (matrices.view != currentMatrix) {
			updated = true;
		}
	}

	void setPosition(glm::vec3 position)
	{
		this->position = position;
		updateViewMatrix();
	}

	void setRotation(glm::vec3 rotation)
	{
		this->rotation = rotation;
		updateViewMatrix();
	}

	void rotate(glm::vec3 delta)
	{
		this->rotation += delta;
		updateViewMatrix();
	}

	void setTranslation(glm::vec3 translation)
	{
		this->position = translation;
		updateViewMatrix();
	};

	void translate(glm::vec3 delta)
	{
		this->position += delta;
		updateViewMatrix();
	}

	void magnify(short wheelDelta) {
		if (orthographic) {
			wheelDelta = -wheelDelta;
			float btm = (orthoBottom + orthoTop) / 2.f;
			float lfm = (orthoLeft + orthoRight) / 2.f;
			orthoTop += (float)wheelDelta * 0.0005f;
			orthoBottom -= (float)wheelDelta * 0.0005f;
			orthoRight += (float)wheelDelta * 0.0005f * aspect;
			orthoLeft -= (float)wheelDelta * 0.0005f * aspect;
			if (orthoTop <= btm) {
				orthoTop = 0.f;
				orthoBottom = 0.f;
			}
			if (orthoRight <= lfm) {
				orthoRight = 0.f;
				orthoLeft = 0.f;
			}
		}
		else {
			translate(getCamFront() * (float)wheelDelta * 0.005f);
		}
	}

	void setRotationSpeed(float rotationSpeed)
	{
		this->rotationSpeed = rotationSpeed;
	}

	void setMovementSpeed(float movementSpeed)
	{
		this->movementSpeed = movementSpeed;
	}

	glm::vec3 getCamFront() {
		glm::vec3 camFront;  //point at camera -z axis
		camFront.x = cos(glm::radians(rotation.x)) * -sin(glm::radians(rotation.y));
		camFront.y = sin(glm::radians(rotation.x));
		camFront.z = cos(glm::radians(rotation.x)) * -cos(glm::radians(rotation.y));
		camFront = glm::normalize(camFront);
		return camFront;
	}

	glm::vec3 getCamRight() {
		//return glm::inverse(glm::mat3(matrices.view)) * glm::vec3(1, 0, 0);
		return glm::normalize(glm::cross(getCamFront(), glm::vec3(0.0f, 1.0f, 0.0f)));
	}

	glm::vec3 getCamUp() {
		//return glm::inverse(glm::mat3(matrices.view)) * glm::vec3(0, 1, 0);
		return glm::cross(getCamRight(), getCamFront());
	}

	void update(float deltaTime)
	{
		updated = false;
		if (type == CameraType::firstperson)
		{
			if (moving())
			{
				glm::vec3 camFront = getCamFront();

				float moveSpeed = deltaTime * movementSpeed;

				if (keys.up)
					position += camFront * moveSpeed;
				if (keys.down)
					position -= camFront * moveSpeed;
				if (keys.left)
					position -= glm::normalize(glm::cross(camFront, glm::vec3(0.0f, 1.0f, 0.0f))) * moveSpeed;
				if (keys.right)
					position += glm::normalize(glm::cross(camFront, glm::vec3(0.0f, 1.0f, 0.0f))) * moveSpeed;
			}
		}
		updateViewMatrix();
	};

	// Update camera passing separate axis data (gamepad)
	// Returns true if view or position has been changed
	bool updatePad(glm::vec2 axisLeft, glm::vec2 axisRight, float deltaTime)
	{
		bool retVal = false;

		if (type == CameraType::firstperson)
		{
			// Use the common console thumbstick layout		
			// Left = view, right = move

			const float deadZone = 0.0015f;
			const float range = 1.0f - deadZone;

			glm::vec3 camFront = getCamFront();

			float moveSpeed = deltaTime * movementSpeed * 2.0f;
			float rotSpeed = deltaTime * rotationSpeed * 50.0f;
			 
			// Move
			if (fabsf(axisLeft.y) > deadZone)
			{
				float pos = (fabsf(axisLeft.y) - deadZone) / range;
				position -= camFront * pos * ((axisLeft.y < 0.0f) ? -1.0f : 1.0f) * moveSpeed;
				retVal = true;
			}
			if (fabsf(axisLeft.x) > deadZone)
			{
				float pos = (fabsf(axisLeft.x) - deadZone) / range;
				position += glm::normalize(glm::cross(camFront, glm::vec3(0.0f, 1.0f, 0.0f))) * pos * ((axisLeft.x < 0.0f) ? -1.0f : 1.0f) * moveSpeed;
				retVal = true;
			}

			// Rotate
			if (fabsf(axisRight.x) > deadZone)
			{
				float pos = (fabsf(axisRight.x) - deadZone) / range;
				rotation.y += pos * ((axisRight.x < 0.0f) ? -1.0f : 1.0f) * rotSpeed;
				retVal = true;
			}
			if (fabsf(axisRight.y) > deadZone)
			{
				float pos = (fabsf(axisRight.y) - deadZone) / range;
				rotation.x -= pos * ((axisRight.y < 0.0f) ? -1.0f : 1.0f) * rotSpeed;
				retVal = true;
			}
		}
		else
		{
			// todo: move code from example base class for look-at
		}

		if (retVal)
		{
			updateViewMatrix();
		}

		return retVal;
	}

};
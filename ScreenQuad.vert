#version 450 core

const vec2 frame_pos[4] = vec2[4] (
	vec2(-1.0f, 1.0f),
	vec2(-1.0f, -1.0f),
	vec2(1.0f, 1.0f),
	vec2(1.0f, -1.0f)
);

/*const vec2 frame_uv[4] = vec2[4] (
	vec2(0.0f, 1.0f),
	vec2(0.0f, 0.0f),
	vec2(1.0, 1.0),
	vec2(1.0, 0.0f)
);*/

void main(){
    gl_Position = vec4(frame_pos[gl_VertexID], 0.0f, 1.0f);
    // uv = frame_uv[gl_vertexID];
}
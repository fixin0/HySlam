#vertex
#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
uniform mat4 uMVP;
uniform vec3 uCameraPos;
uniform float uPointSize;
uniform float uNearFade;
uniform float uFarFade;
out vec3 vertexColor;
void main()
{
    vec4 worldPos = vec4(aPosition, 1.0);
    gl_Position = uMVP * worldPos;
    gl_PointSize = uPointSize;
    float dist = length(aPosition - uCameraPos);
    float fade = 1.0;
    if (dist > uNearFade && uFarFade > uNearFade) {
        fade *= 1.0 - clamp((dist - uNearFade) / (uFarFade - uNearFade), 0.0, 1.0);
    }
    if (dist < uNearFade) {
        fade *= clamp(dist / uNearFade, 0.0, 1.0);
    }
    gl_PointSize *= clamp(15.0 / (dist + 1.0), 0.5, 2.0);
    vertexColor = aColor * fade;
}

#fragment
#version 330 core
in vec3 vertexColor;
uniform int uRoundPoints;
out vec4 FragColor;
void main()
{
    if (uRoundPoints == 1) {
        vec2 coord = gl_PointCoord * 2.0 - 1.0;
        if (dot(coord, coord) > 1.0) {
            discard;
        }
    }
    FragColor = vec4(vertexColor, 1.0);
}

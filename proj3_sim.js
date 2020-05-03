/*
 * Global variables
 */
var meshResolution;

// Particle states
var mass;
var vertexPosition, vertexNormal;
var vertexVelocity;

// Spring properties
var K, restLength;

// Force parameters
var Cd;
var uf, Cv;


/*
 * Getters and setters
 */
function getPosition(i, j) {
    var id = i * meshResolution + j;
    return vec3.create([vertexPosition[3 * id], vertexPosition[3 * id + 1], vertexPosition[3 * id + 2]]);
}

function setPosition(i, j, x) {
    var id = i * meshResolution + j;
    vertexPosition[3 * id] = x[0]; vertexPosition[3 * id + 1] = x[1]; vertexPosition[3 * id + 2] = x[2];
}

function getNormal(i, j) {
    var id = i * meshResolution + j;
    return vec3.create([vertexNormal[3 * id], vertexNormal[3 * id + 1], vertexNormal[3 * id + 2]]);
}

function getVelocity(i, j) {
    var id = i * meshResolution + j;
    return vec3.create(vertexVelocity[id]);
}

function setVelocity(i, j, v) {
    var id = i * meshResolution + j;
    vertexVelocity[id] = vec3.create(v);
}


/*
 * Provided global functions (you do NOT have to modify them)
 */
function computeNormals() {
    var dx = [1, 1, 0, -1, -1, 0], dy = [0, 1, 1, 0, -1, -1];
    var e1, e2;
    var i, j, k = 0, t;
    for (i = 0; i < meshResolution; ++i)
        for (j = 0; j < meshResolution; ++j) {
            var p0 = getPosition(i, j), norms = [];
            for (t = 0; t < 6; ++t) {
                var i1 = i + dy[t], j1 = j + dx[t];
                var i2 = i + dy[(t + 1) % 6], j2 = j + dx[(t + 1) % 6];
                if (i1 >= 0 && i1 < meshResolution && j1 >= 0 && j1 < meshResolution &&
                    i2 >= 0 && i2 < meshResolution && j2 >= 0 && j2 < meshResolution) {
                    e1 = vec3.subtract(getPosition(i1, j1), p0);
                    e2 = vec3.subtract(getPosition(i2, j2), p0);
                    norms.push(vec3.normalize(vec3.cross(e1, e2)));
                }
            }
            e1 = vec3.create();
            for (t = 0; t < norms.length; ++t) vec3.add(e1, norms[t]);
            vec3.normalize(e1);
            vertexNormal[3 * k] = e1[0];
            vertexNormal[3 * k + 1] = e1[1];
            vertexNormal[3 * k + 2] = e1[2];
            ++k;
        }
}


var clothIndex, clothWireIndex;
function initMesh() {
    var i, j, k;

    vertexPosition = new Array(meshResolution * meshResolution * 3);
    vertexNormal = new Array(meshResolution * meshResolution * 3);
    clothIndex = new Array((meshResolution - 1) * (meshResolution - 1) * 6);
    clothWireIndex = [];

    vertexVelocity = new Array(meshResolution * meshResolution);
    restLength[0] = 4.0 / (meshResolution - 1);
    restLength[1] = Math.sqrt(2.0) * 4.0 / (meshResolution - 1);
    restLength[2] = 2.0 * restLength[0];

    for (i = 0; i < meshResolution; ++i)
        for (j = 0; j < meshResolution; ++j) {
            setPosition(i, j, [-2.0 + 4.0 * j / (meshResolution - 1), -2.0 + 4.0 * i / (meshResolution - 1), 0.0]);
            setVelocity(i, j, vec3.create());

            if (j < meshResolution - 1)
                clothWireIndex.push(i * meshResolution + j, i * meshResolution + j + 1);
            if (i < meshResolution - 1)
                clothWireIndex.push(i * meshResolution + j, (i + 1) * meshResolution + j);
            if (i < meshResolution - 1 && j < meshResolution - 1)
                clothWireIndex.push(i * meshResolution + j, (i + 1) * meshResolution + j + 1);
        }
    computeNormals();

    k = 0;
    for (i = 0; i < meshResolution - 1; ++i)
        for (j = 0; j < meshResolution - 1; ++j) {
            clothIndex[6 * k] = i * meshResolution + j;
            clothIndex[6 * k + 1] = i * meshResolution + j + 1;
            clothIndex[6 * k + 2] = (i + 1) * meshResolution + j + 1;
            clothIndex[6 * k + 3] = i * meshResolution + j;
            clothIndex[6 * k + 4] = (i + 1) * meshResolution + j + 1;
            clothIndex[6 * k + 5] = (i + 1) * meshResolution + j;
            ++k;
        }
}


function springForce(i, j, other_i, other_j, type) {
    let p1 = getPosition(i, j);
    let p2 = getPosition(other_i, other_j);
    vec3.subtract(p1, p2);
    let length = vec3.length(p1);
    let scalar = K[type] * (restLength[type] - length) * (1/length);

    return vec3.scale(p1, scalar);
}

function viscousForce(i, j) {
    let dot = vec3.dot(
        getNormal(i, j),
        vec3.subtract(
            vec3.create(uf),
            getVelocity(i, j)
        )
    );

    return vec3.scale(
        getNormal(i, j),
        dot * Cv
    );
}

function getSpringForces(i, j) {
    // add F_spring for all 12 springs
    let springForces = [];
    if (i > 0) {
        // down
        springForces.push([i - 1, j, 0]);
    }
    if (i > 1) {
        // down 2
        springForces.push([i - 2, j, 2]);
    }
    if (j > 0) {
        // left
        springForces.push([i, j - 1, 0]);
    }
    if (j > 1) {
        // left 2
        springForces.push([i, j - 2, 2]);
    }
    if (j < meshResolution - 1) {
        // right
        springForces.push([i, j + 1, 0]);
    }
    if (j < meshResolution - 2) {
        // right 2
        springForces.push([i, j + 2, 2]);
    }
    if (i < meshResolution - 1) {
        // up
        springForces.push([i + 1, j, 0]);
    }
    if (i < meshResolution - 2) {
        // up 2
        springForces.push([i + 2, j, 2]);
    }
    if (i > 0 && j > 0) {
        // diag left down
        springForces.push([i - 1, j - 1, 1]);
    }
    if (i > 0 && j < meshResolution - 1) {
        // diag right down
        springForces.push([i - 1, j + 1, 1]);
    }
    if (i < meshResolution - 1 && j < meshResolution - 1) {
        // diag right up
        springForces.push([i + 1, j + 1, 1]);
    }
    if (j > 0 && i < meshResolution - 1) {
        // diag left up
        springForces.push([i + 1, j - 1, 1]);
    }

    let forces = springForces.map((args) => springForce(i, j, ...args));
    let total_force = vec3.create();
    for (let force of forces) {
        vec3.add(
            total_force,
            force
        )
    }

    return total_force
}

function simulate(stepSize) {
    // for each particle
    for (let i = 0; i < meshResolution; i++) {
        for (let j = 0; j < meshResolution; j++) {
            // pin 2 particles
            if (i == meshResolution - 1 && j == 0) {
                continue;
            }
            if (i == meshResolution - 1 && j == meshResolution - 1) {
                continue;
            }

            let total_force = vec3.create([0, 0, 0]);

            vec3.add(
                total_force,
                getSpringForces(i, j)
            );

            // add gravity
            vec3.add(
                total_force,
                vec3.create([0, -mass * 9.8, 0])
            );

            // add damping
            vec3.add(
                total_force,
                vec3.scale(
                    getVelocity(i, j),
                    -Cd
                )
            );

            // // add viscous fluid
            vec3.add(
                total_force,
                viscousForce(i, j)
            )

            // update v <- v + stepSize*total_force/m
            let new_vel = vec3.add(
                getVelocity(i, j),
                vec3.scale(
                    vec3.scale(
                        total_force,
                        1 / mass
                    ),
                    stepSize
                )
            );
            setVelocity(i, j, new_vel);

            // update x <- x + stepSize*v
            let new_pos = vec3.add(
                getPosition(i, j),
                vec3.scale(
                    getVelocity(i, j),
                    stepSize
                )
            );

            setPosition(i, j, new_pos);
        }
    }

}
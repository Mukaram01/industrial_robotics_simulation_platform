let scene, camera, renderer, controls;
const objects = [];
const scenario = { robots: [], objects: [] };

function init() {
    const container = document.getElementById('canvas-container');
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(75, container.clientWidth / 500, 0.1, 1000);
    camera.position.set(3, 3, 3);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({antialias: true});
    renderer.setSize(container.clientWidth, 500);
    container.appendChild(renderer.domElement);

    const grid = new THREE.GridHelper(10, 10);
    scene.add(grid);

    addRobot();
    addObject();

    controls = new THREE.DragControls(objects, camera, renderer.domElement);
    controls.addEventListener('dragend', updateData);

    animate();
}

function addRobot() {
    const geo = new THREE.BoxGeometry(0.2, 0.2, 0.2);
    const mat = new THREE.MeshBasicMaterial({color: 0x00ff00});
    const mesh = new THREE.Mesh(geo, mat);
    mesh.userData.type = 'robot';
    mesh.userData.index = scenario.robots.length;
    scene.add(mesh);
    objects.push(mesh);
    scenario.robots.push({id: 'robot_' + scenario.robots.length, position: [0, 0, 0]});
}

function addObject() {
    const geo = new THREE.BoxGeometry(0.2, 0.2, 0.2);
    const mat = new THREE.MeshBasicMaterial({color: 0xff0000});
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(0.5, 0, 0.1);
    mesh.userData.type = 'object';
    mesh.userData.index = scenario.objects.length;
    scene.add(mesh);
    objects.push(mesh);
    scenario.objects.push({id: 'obj_' + scenario.objects.length, position: [0.5, 0, 0.1]});
}

function updateData(evt) {
    const obj = evt.object;
    const pos = [obj.position.x, obj.position.y, obj.position.z];
    if (obj.userData.type === 'robot') {
        scenario.robots[obj.userData.index].position = pos;
    } else {
        scenario.objects[obj.userData.index].position = pos;
    }
}

function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}

function saveScenario() {
    fetch('/api/scenarios/edited', {
        method: 'PUT',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({config: scenario})
    }).then(r => r.json()).then(console.log);
}

document.addEventListener('DOMContentLoaded', () => {
    init();
    document.getElementById('save-btn').addEventListener('click', saveScenario);
});

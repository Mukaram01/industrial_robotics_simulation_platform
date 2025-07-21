var EnvViewer = (function () {
    let scene, camera, renderer, canvas;
    function init(canvasElem) {
        canvas = canvasElem;
        scene = new THREE.Scene();
        const width = canvas.clientWidth;
        const height = canvas.clientHeight;
        camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 100);
        camera.position.set(1.5, 1.5, 1.5);
        camera.lookAt(0, 0, 0);
        renderer = new THREE.WebGLRenderer({ canvas: canvas });
        renderer.setSize(width, height);
        const light = new THREE.DirectionalLight(0xffffff, 1);
        light.position.set(2, 2, 2);
        scene.add(light);
        scene.add(new THREE.GridHelper(2, 10));
        animate();
    }
    function animate() {
        requestAnimationFrame(animate);
        if (renderer && scene && camera) {
            renderer.render(scene, camera);
        }
    }
    function clearScene() {
        if (!scene) return;
        while (scene.children.length > 0) {
            scene.remove(scene.children.pop());
        }
        const light = new THREE.DirectionalLight(0xffffff, 1);
        light.position.set(2, 2, 2);
        scene.add(light);
        scene.add(new THREE.GridHelper(2, 10));
    }
    function buildEnvironment(env) {
        clearScene();
        if (!env) return;
        const material = new THREE.MeshLambertMaterial({ color: 0xcccccc });
        (env.objects || []).forEach(obj => {
            const d = obj.dimensions || [0.1, 0.1, 0.1];
            const geo = new THREE.BoxGeometry(d[0], d[1], d[2]);
            const mesh = new THREE.Mesh(geo, material);
            const p = obj.position || [0, 0, 0];
            mesh.position.set(p[0], p[1], p[2]);
            scene.add(mesh);
        });
        (env.containers || []).forEach(obj => {
            const d = obj.dimensions || [0.1, 0.1, 0.1];
            const geo = new THREE.BoxGeometry(d[0], d[1], d[2]);
            const mesh = new THREE.Mesh(geo, material);
            const p = obj.position || [0, 0, 0];
            mesh.position.set(p[0], p[1], p[2]);
            scene.add(mesh);
        });
        (env.conveyors || []).forEach(obj => {
            const d = obj.dimensions || [0.1, 0.1, 0.1];
            const geo = new THREE.BoxGeometry(d[0], d[1], d[2]);
            const mesh = new THREE.Mesh(geo, material);
            const p = obj.position || [0, 0, 0];
            mesh.position.set(p[0], p[1], p[2]);
            scene.add(mesh);
        });
    }
    function updateObjects(objects) {
        if (!scene) return;
        const material = new THREE.MeshNormalMaterial();
        objects.forEach(obj => {
            const geo = new THREE.BoxGeometry(0.05, 0.05, 0.05);
            const mesh = new THREE.Mesh(geo, material);
            mesh.position.set(obj.position.x, obj.position.y, obj.position.z);
            scene.add(mesh);
        });
    }
    function loadEnvironment() {
        fetch('/api/environment').then(r => r.json()).then(data => {
            buildEnvironment(data.environment);
        }).catch(() => {});
    }
    return { init, loadEnvironment, updateObjects };
})();

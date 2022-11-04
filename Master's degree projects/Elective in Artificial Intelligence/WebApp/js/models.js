import * as THREE from 'https://threejs.org/build/three.module.js';
import * as GAME from './game.js'

export let disks = [];
export let rods = [];
export let cylinderHeight; // = GAME.canvas.height * 20 / 100;
let floor_height;
let disk_height;
export let thickness; // = GAME.canvas.width * 0.7 / 100; // = GAME.canvas.width * 0.7 / 100;

// LIGHTS
export function addLights(scene) {
  // hemisphere light
  const skyColor = 0xB1E1FF;  // light blue
  const groundColor = 0xB97A20;  // brownish orange
  const color = 0xFFFFFF; // white

  let intensity_hem = 1;

  const hemisphere_light = new THREE.AmbientLight(skyColor, intensity_hem);
  hemisphere_light.name = "Hemisphere Light";
  scene.add(hemisphere_light);


  let intensity_dir = 1;
  // directional light
  const directional_light = new THREE.DirectionalLight(color, intensity_dir);
  directional_light.position.set(100, 100, 100);
  directional_light.castShadow = true;
  directional_light.shadow.mapSize.width = 100 * window.devicePixelRatio;
  directional_light.shadow.mapSize.height = 100 * window.devicePixelRatio;
  directional_light.shadow.camera = new THREE.OrthographicCamera( -100, 100, 100, -100, 0.1, 3000); 
  directional_light.shadow.camera.near = 0.1;
  directional_light.shadow.camera.far = 3000;
  directional_light.shadow.camera.fov = 1000;
  directional_light.name = "Directional Light";

  scene.add(directional_light);
}

export function create_rods(model) {
  thickness= GAME.canvas.width * 0.7 / 100;
  const radiusTop = 1;
  const radiusBottom = 1;
  cylinderHeight = GAME.canvas.height * 20 / 100;
  const baseSides = 100;
  const geometry = new THREE.CylinderGeometry(radiusTop, radiusBottom, cylinderHeight, baseSides);

  let material = new THREE.MeshPhongMaterial({color: 0xB97A20});

  const rod1 = new THREE.Mesh(geometry, material);
  rod1.position.set(-GAME.canvas.width * 9 / 100, 3, 0);
  rod1.receiveShadow = true;
  rod1.name = "rod1";
  model.add(rod1);

  const rod2 = new THREE.Mesh(geometry, material);
  rod2.position.set(0, 3, 0);
  rod2.receiveShadow = true;
  rod2.name = "rod2";
  model.add(rod2);

  const rod3 = new THREE.Mesh(geometry, material);
  rod3.position.set(GAME.canvas.width * 9 / 100, 3, 0);
  rod3.receiveShadow = true;
  rod3.name = "rod3";
  model.add(rod3);

  floor_height = GAME.canvas.height * 0.5 / 100;
  let width = GAME.canvas.width * 30 / 100;
  let depth = GAME.canvas.width * 15 / 100;

  const geometry_floor = new THREE.BoxGeometry(floor_height, width, depth);
  let material_floor = new THREE.MeshPhongMaterial({color: 0xB97A20});

  const floor = new THREE.Mesh(geometry_floor, material_floor);
  floor.position.set(0, - cylinderHeight/2 + 3, 0);
  floor.rotation.z = Math.PI/2;
  floor.receiveShadow = true;
  floor.name = "floor";
  model.add(floor);
}


export function createDisks(model, level) {
  const radial_segment = 50; // prima 15
  const tab_segment = 100; // prima 70

  let material1, material2, material3, material4, material5, material6, material7;

  //ROSSO
  const radius1 = GAME.canvas.width * 0.5 / 100;
  const geometry1 = new THREE.TorusGeometry(radius1, thickness, radial_segment, tab_segment);
  material1 = new THREE.MeshPhongMaterial({color: 0xF20000});
  const disk1 = new THREE.Mesh(geometry1, material1);
  disk1.rotation.x = Math.PI/2;
  disk1.receiveShadow = true;
  disk1.name = "disk1";

  //BLU
  const radius2 = GAME.canvas.width * 1 / 100;
  const geometry2 = new THREE.TorusGeometry(radius2, thickness, radial_segment, tab_segment);
  material2 = new THREE.MeshPhongMaterial({color: 0x2929FF});
  const disk2 = new THREE.Mesh(geometry2, material2);
  disk2.rotation.x = Math.PI/2;
  disk2.receiveShadow = true;
  disk2.name = "disk2";

  //VERDE
  const radius3 = GAME.canvas.width * 1.5 / 100;
  const geometry3 = new THREE.TorusGeometry(radius3, thickness, radial_segment, tab_segment);
  material3 = new THREE.MeshPhongMaterial({color: 0x00DB00});
  const disk3 = new THREE.Mesh(geometry3, material3);
  disk3.rotation.x = Math.PI/2;
  disk3.receiveShadow = true;
  disk3.name = "disk3";

  if (level==1){
    disk1.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*5 - cylinderHeight/2, 0);
    model.add(disk1);
    disk2.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*3 - cylinderHeight/2, 0);
    model.add(disk2);
    disk3.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness - cylinderHeight/2, 0);
    model.add(disk3);
  }else if(level>=2){
    //ROSA
    const radius4 = GAME.canvas.width * 2 / 100;
    const geometry4 = new THREE.TorusGeometry(radius4, thickness, radial_segment, tab_segment);
    material4 = new THREE.MeshPhongMaterial({color: 0xFF0DFF});
    const disk4 = new THREE.Mesh(geometry4, material4);
    disk4.rotation.x = Math.PI/2;
    disk4.receiveShadow = true;
    disk4.name = "disk4";

    //GIALLO
    const radius5 = GAME.canvas.width * 2.5 / 100;
    const geometry5 = new THREE.TorusGeometry(radius5, thickness, radial_segment, tab_segment);
    material5 = new THREE.MeshPhongMaterial({color: 0xFFFF00});
    const disk5 = new THREE.Mesh(geometry5, material5);
    disk5.rotation.x = Math.PI/2;
    disk5.receiveShadow = true;
    disk5.name = "disk5";

    if(level==2){
      disk1.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*9 - cylinderHeight/2, 0);
      model.add(disk1);
      disk2.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*7 - cylinderHeight/2, 0);
      model.add(disk2);
      disk3.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*5 - cylinderHeight/2, 0);
      model.add(disk3);
      disk4.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*3 - cylinderHeight/2, 0);    
      model.add(disk4);
      disk5.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness - cylinderHeight/2, 0);
      model.add(disk5);
    }else if (level==3){
        //ARANCIONE
        const radius6 = GAME.canvas.width * 3 / 100;
        const geometry6 = new THREE.TorusGeometry(radius6, thickness, radial_segment, tab_segment);
        material6 = new THREE.MeshPhongMaterial({color: 0xFF4300});
        const disk6 = new THREE.Mesh(geometry6, material6);
        disk6.rotation.x = Math.PI/2;
        disk6.receiveShadow = true;
        disk6.name = "disk6";

        //BIANCO
        const radius7 = GAME.canvas.width * 3.5 / 100;
        material7 = new THREE.MeshPhongMaterial({color: 0xFFFFFF});
        const geometry7 = new THREE.TorusGeometry(radius7, thickness, radial_segment, tab_segment);
        const disk7 = new THREE.Mesh(geometry7, material7);
        disk7.rotation.x = Math.PI/2;
        disk7.receiveShadow = true;
        disk7.name = "disk7";

        disk1.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*13 - cylinderHeight/2, 0);
        model.add(disk1);
        disk2.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*11 - cylinderHeight/2, 0);
        model.add(disk2);
        disk3.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*9 - cylinderHeight/2, 0);
        model.add(disk3);
        disk4.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*7 - cylinderHeight/2, 0);
        model.add(disk4);
        disk5.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*5 - cylinderHeight/2, 0);
        model.add(disk5);
        disk6.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness*3 - cylinderHeight/2, 0);
        model.add(disk6);
        disk7.position.set(-GAME.canvas.width * 9 / 100, 3 + thickness - cylinderHeight/2, 0);
        model.add(disk7);

    }
  }

  if(level==1){

  }else if(level==2){

  }else if(level==3){

  }
  
}

export function createWorld(scene, model, level) {
  addLights(scene);

  create_rods(model);

  createDisks(model, level);

  //createCylinder(model, level);

  //createHierarchicalModel(model, level);

  //OBJECTS.add_all_objects_in_world(scene, model, level);
}
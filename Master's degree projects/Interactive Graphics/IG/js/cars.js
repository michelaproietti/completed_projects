import * as THREE from 'https://threejs.org/build/three.module.js';
import * as WORLD from './models.js'
import * as BUNNY from './character_bunny.js'
import * as BEAR from './character_bear.js'
import * as GIRL from './character_man.js'
import * as GAME from './game.js'

export let dead_hero = false;

export function animate_cars() {
    for (let car of WORLD.cars_ltr) {
        car.position.x += 0.3;
        if (car.position.x > 25) car.position.x = -25;
    }
    for (let car of WORLD.cars_rtl) {
        car.position.x -= 0.2;
        if (car.position.x < -25) car.position.x = 25;
    }
}

export function check_collisions(level) {
    let carPos = new THREE.Vector3();
    let hero_position;
    if (level == 1) hero_position = BUNNY.hero_position;
    else if (level == 2) hero_position = BEAR.hero_position;
    else if (level == 3) hero_position = GIRL.hero_position;

    WORLD.cars.forEach(function (car_ith, index_car_ith) {
        let car = WORLD.cars[index_car_ith];
        carPos.setFromMatrixPosition(car.matrixWorld);
        if(carPos.z >= -14 && carPos.z <= 1 && carPos.y > 0 && carPos.x > hero_position.x - 1 && carPos.x < hero_position.x + 1){ // check just for cars that are in the next slice, up to 7.5 deg (hero.position.z = 0 always)
            let distancex = Math.abs(hero_position.x-carPos.x);
            let distancez = Math.abs(carPos.z);
            if(((distancex <= 1 && distancez <= 0.15) || (distancex <= 1 && distancez < 3.5)) && !dead_hero){
                window.cancelAnimationFrame(GAME.id);
                GAME.sound_audio.stop();
                GAME.sound_audio2.play();
                dead_hero = true;
            }
        }
      });
}
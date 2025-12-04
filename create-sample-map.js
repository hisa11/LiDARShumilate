const Jimp = require('jimp');

async function createSampleMap() {
    const width = 400;
    const height = 300;
    
    // 白い背景（空きスペース）
    const image = new Jimp(width, height, 0xFFFFFFFF);
    
    // 黒で壁を描画（0x000000FF）
    const black = 0x000000FF;
    
    // 外壁
    for (let x = 0; x < width; x++) {
        image.setPixelColor(black, x, 0);
        image.setPixelColor(black, x, height - 1);
    }
    for (let y = 0; y < height; y++) {
        image.setPixelColor(black, 0, y);
        image.setPixelColor(black, width - 1, y);
    }
    
    // 部屋1（左上）
    for (let x = 20; x < 120; x++) {
        for (let y = 20; y < 22; y++) image.setPixelColor(black, x, y);
        for (let y = 100; y < 102; y++) image.setPixelColor(black, x, y);
    }
    for (let y = 20; y < 100; y++) {
        for (let x = 20; x < 22; x++) image.setPixelColor(black, x, y);
        for (let x = 118; x < 120; x++) image.setPixelColor(black, x, y);
    }
    
    // 部屋2（右上）
    for (let x = 200; x < 300; x++) {
        for (let y = 20; y < 22; y++) image.setPixelColor(black, x, y);
        for (let y = 100; y < 102; y++) image.setPixelColor(black, x, y);
    }
    for (let y = 20; y < 100; y++) {
        for (let x = 200; x < 202; x++) image.setPixelColor(black, x, y);
        for (let x = 298; x < 300; x++) image.setPixelColor(black, x, y);
    }
    
    // 廊下の壁
    for (let x = 50; x < 250; x++) {
        for (let y = 140; y < 142; y++) image.setPixelColor(black, x, y);
        for (let y = 160; y < 162; y++) image.setPixelColor(black, x, y);
    }
    
    // 柱
    for (let x = 150; x < 160; x++) {
        for (let y = 200; y < 210; y++) {
            image.setPixelColor(black, x, y);
        }
    }
    
    await image.writeAsync('sample-map.png');
    console.log('Sample map created: sample-map.png');
}

createSampleMap().catch(console.error);

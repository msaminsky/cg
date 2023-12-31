var values = {
  amount: 15,
};

var raster, group;
var piece = createPiece();
var count = 0;

handleImage("instaIcon");

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function createPiece() {
  var group = new Group();
  var hexagon = new Path.RegularPolygon({
    center: view.center,
    sides: 6,
    radius: 50,
    fillColor: "gray",
    parent: group,
  });
  for (var i = 0; i < 2; i++) {
    var path = new Path({
      closed: true,
      parent: group,
      fillColor: i == 0 ? "white" : "black",
    });
    for (var j = 0; j < 3; j++) {
      var index = (i * 2 + j) % hexagon.segments.length;
      path.add(hexagon.segments[index].clone());
    }
    path.add(hexagon.bounds.center);
  }
  // Remove the group from the document, so it is not drawn:
  group.remove();
  return group;
}

function handleImage(image) {
  count = 0;
  var size = piece.bounds.size;

  if (group) group.remove();

  // As the web is asynchronous, we need to wait for the raster to
  // load before we can perform any operation on its pixels.
  raster = new Raster(image);
  raster.visible = false;
  raster.on("load", async function () {
    // Transform the raster, so it fills the view:
    raster.fitBounds(view.bounds, true);
    group = new Group();
    for (var y = 0; y < values.amount; y++) {
      for (var x = 0; x < values.amount; x++) {
        var copy = piece.clone();
        copy.position += size * [x + (y % 2 ? 0.5 : 0), y * 0.75];
        group.addChild(copy);
      }
    }

    // Transform the group so it covers the view:
    group.fitBounds(view.bounds, true);
    group.scale(1.1);
  });
}

async function onFrame() {
  if (!group) return;

  // Loop through the uncolored hexagons in the group and fill them
  // with the average color:
  var length = Math.min(count + values.amount, group.children.length);
  for (var i = count; i < length; i++) {
    await sleep(8);
    piece = group.children[i];
    var hexagon = piece.children[0];
    var color = raster.getAverageColor(hexagon);
    console.log("init", {
      i,
      color,
      right: piece.children[2],
      top: piece.children[1],
    });
    if (color) {
      hexagon.fillColor = color;
      var top = piece.children[1];
      top.fillColor = color.clone();
      top.fillColor.brightness *= 1.5;

      var right = piece.children[2];
      right.fillColor = color.clone();
      right.fillColor.brightness *= 0.5;
    }
    console.log("after", { i, color, right, top, chidlren: piece.children[2] });
  }
  count += values.amount;
}

function onDocumentMouseOver() {
  console.log("hello");

  handleImage("instaIcon");
}

var instaIcon = document.getElementById("insta-container");
instaIcon.addEventListener("mouseover", onDocumentMouseOver, false);

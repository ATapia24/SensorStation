<?php

	$servername = "";
	$username = "";
	$password = "";
	$dbname = "";
	$key = "";
	
	if (strcmp($_GET["key"], $key) != 0) {
		echo "<img src=\"./images/sorrydave.jpg\"></img>";
		die();
	}


	$conn = new mysqli($servername, $username, $password, $dbname);
	
	if ($conn->connect_error) {
		die("Connection failed: " . $conn->connect_error);
	} else {
		echo "connection established<br />";
	}
	
	$sql = "INSERT INTO weather (temperature, humidity, humidity_temp, rssi, compass_x, compass_y, compass_z, compass_heading, uv, gas, rgb_r, rgb_g, rgb_b, rgb_lux, rgb_intensity, accel_x, accel_y, accel_z, pressure, pressure_altitude, pressure_temp) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)";

	$stmt = $conn->prepare($sql);
	$stmt->bind_param("iddiddddiiiiiiiiiiddd", $temperature, $hum, $humtemp, $rssi, $cx, $cy, $cz, $ch, $uv, $gas, $red, $green, $blue, $lux, $int, $x, $y, $z, $press, $alt, $press_temp);

	$temperature = $_GET["t"];
	$rssi = $_GET["rssi"];
	$x = $_GET["x"];
	$y = $_GET["y"];
	$z = $_GET["z"];
	$cx = $_GET["cx"];
	$cy = $_GET["cy"];
	$cz = $_GET["cz"];
	$ch = $_GET["ch"];
	$red = $_GET["r"];
	$green = $_GET["g"];
	$blue = $_GET["b"];
	$lux = $_GET["lux"];
	$int = $_GET["int"];
	$hum = $_GET["hum"];
	$humtemp = $_GET["humtemp"];
	$press = $_GET["press"];
	$alt = $_GET["alt"];
	$press_temp = $_GET["press_temp"];
	$uv = $_GET["uv"];
	$gas = $_GET["gas"];

	// basic checks
	if ($temperature === null) {
		$temperature = 0.0;
	}
	if ($rssi === null) {
		$rssi = 0;
	}
	if ($hum == null) {
		$hum = 0;
	}
	if ($humtemp == null) {
		$humtemp = 0;
	}
	if ($cx == null) {
		$cx = 0;
	}
	if ($cy == null) {
		$cy = 0;
	}
	if ($cz == null) {
		$cz = 0;
	}
	if ($ch == null) {
		$ch = 0;
	}
	if ($uv == null) {
		$uv = 0;
	}
	if ($gas == null) {
		$gas = 0;
	}
	if ($red == null) {
		$red = 0;
	}
	if ($blue == null) {
		$blue = 0;
	}
	if ($green == null) {
		$green = 0;
	}
	if ($cx == null) {
		$cx = 0;
	}
	if ($lux == null) {
		$lux = 0;
	}
	if ($int == null) {
		$int = 0;
	}
	if ($press == null) {
		$press = 0;
	}
	if ($alt == null) {
		$alt = 0;
	}
	if ($press_temp == null) {
		$press_temp = 0;
	}

	if ($stmt->execute()) {
		echo "Data has been posted to the database\n";
	} else {
		echo "There was an error in posting the data: " . $conn->error;
	}
	
	$stmt->close();
	$conn->close();
	
?>
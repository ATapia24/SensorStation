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
	
	$sql = "INSERT INTO weather (temperature, orientation, rssi) VALUES (?, ?, ?)";

	$stmt = $conn->prepare($sql);
	$stmt->bind_param("dsi", $temperature, $orientation, $rssi);

	$orientation = "x: " . $_GET["x"] . " y: " . $_GET["y"] . " z: " . $_GET["z"];
	$temperature = $_GET["t"];
	$rssi = $_GET["rssi"];

	// basic checks
	if ($temperature === null) {
		$temperature = 0.0;
	}
	if ($rssi === null) {
		$rssi = 0;
	}

	if ($stmt->execute()) {
		echo "Data has been posted to the database\n";
	} else {
		echo "There was an error in posting the data: " . $conn->error;
	}
	
	$stmt->close();
	$conn->close();
	
?>
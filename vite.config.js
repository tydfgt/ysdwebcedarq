import { defineConfig } from "vite";

export default defineConfig({
	server: {
		host: "0.0.0.0",
		allowedHosts: [
			"cedarq.cloud",
			"www.cedarq.cloud",
			"localhost",
			"10.1.12.16",
			"43.139.97.198",
		],
	},
	build: {
		assetsInlineLimit: 4096,
	},
});


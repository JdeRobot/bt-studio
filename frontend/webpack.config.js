const MiniCssExtractPlugin = require("mini-css-extract-plugin");
const BundleTrackerPlugin = require("webpack-bundle-tracker");
const path = require("path");

const aliases = () => {
  const aliasConfig = {
    BtApi: path.resolve(__dirname, "src/api_helper"),
    BtComponents: path.resolve(__dirname, "src/components"),
    BtContexts: path.resolve(__dirname, "src/contexts"),
    BtHooks: path.resolve(__dirname, "src/hooks"),
    BtStyles: path.resolve(__dirname, "src/styles"),
    BtTypes: path.resolve(__dirname, "src/types"),
    BtTemplates: path.resolve(__dirname, "src/templates"),
    BtRoutes: path.resolve(__dirname, "src/routes/index.ts"),
    BtIcons: path.resolve(__dirname, "src/icons"),
  };

  return aliasConfig;
};

module.exports = {
  entry: {
    main: "./src/index.js",
  },
  output: {
    filename: "js/[name].[contenthash:8].js",
    clean: true,
  },
  resolve: {
    alias: aliases(),
    extensions: [".js", ".jsx", ".ts", ".tsx", ".json"],
    modules: ["node_modules", path.resolve(__dirname, "node_modules")],
  },
  module: {
    rules: [
      {
        test: /\.css$/i,
        use: ["style-loader", "css-loader"],
      },
      {
        test: /\.(js|jsx|ts|tsx)$/,
        exclude: /node_modules/,
        use: [
          {
            loader: "babel-loader",
            options: {
              presets: [
                "@babel/preset-env",
                ["@babel/preset-react", { runtime: "automatic" }],
                "@babel/preset-typescript",
              ],
            },
          },
        ],
      },
      {
        test: /\.(png|jpg|gif)$/,
        type: "asset/resource",
      },
      {
        test: /\.svg$/i,
        use: [
          {
            loader: "@svgr/webpack",
            options: {
              svgoConfig: {
                plugins: [{ name: "preset-default", removeViewBox: false }],
              },
            },
          },
          "file-loader",
        ],
      },
    ],
  },
  plugins: [
    new BundleTrackerPlugin({
      path: "./",
      filename: "webpack-stats.json",
    }),
  ],
  devtool: "source-map",
};

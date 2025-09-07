const MiniCssExtractPlugin = require("mini-css-extract-plugin");
const BundleTrackerPlugin = require("webpack-bundle-tracker");
const path = require("path");

const aliases = () => {
  const aliasConfig = {
    Api: path.resolve(__dirname, "src/api_helper"),
    Components: path.resolve(__dirname, "src/components"),
    Contexts: path.resolve(__dirname, "src/contexts"),
    Hooks: path.resolve(__dirname, "src/hooks"),
    Styles: path.resolve(__dirname, "src/styles"),
    Types: path.resolve(__dirname, "src/types"),
    Templates: path.resolve(__dirname, "src/templates"),
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

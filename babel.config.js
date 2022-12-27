const math = require('remark-math');
const katex = require('rehype-katex');

module.exports = {
  presets: [require.resolve('@docusaurus/core/lib/babel/preset')],
  remarkPlugins: [math],
  rehypePlugins: [katex],
};

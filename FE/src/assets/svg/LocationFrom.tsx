import type { SVGProps } from 'react';
const SvgLocationFrom = (props: SVGProps<SVGSVGElement>) => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    fill="none"
    viewBox="0 0 24 63"
    {...props}
  >
    <path
      fill="#E0E0E0"
      fillRule="evenodd"
      d="M17.94 4.86a8.4 8.4 0 0 1 0 11.88L12 22.68l-5.94-5.94A8.4 8.4 0 1 1 17.94 4.86ZM12 13.2a2.4 2.4 0 1 0 0-4.8 2.4 2.4 0 0 0 0 4.8Z"
      clipRule="evenodd"
    />
    <path fill="url(#LocationFrom_svg__a)" d="M10 18h4v45h-4z" />
    <defs>
      <linearGradient
        id="LocationFrom_svg__a"
        x1={12}
        x2={12}
        y1={22.939}
        y2={58.61}
        gradientUnits="userSpaceOnUse"
      >
        <stop stopColor="#E0E0E0" />
        <stop offset={1} stopColor="#E0E0E0" stopOpacity={0} />
      </linearGradient>
    </defs>
  </svg>
);
export default SvgLocationFrom;

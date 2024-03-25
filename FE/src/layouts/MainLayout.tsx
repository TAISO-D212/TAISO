interface ILayoutProps {
	children: React.ReactNode;
}

export const MainLayout = ({ children }: ILayoutProps) => {
	return <div className='w-full h-lvh flex justify-center items-center'>{children}</div>;
};

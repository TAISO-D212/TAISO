interface ILayoutProps {
	children: React.ReactNode;
}

export const MainLayout = ({ children }: ILayoutProps) => {
	return <div className='w-full h-dvh'>{children}</div>;
};
